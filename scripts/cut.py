#!/usr/bin/env python
import roslib
import sys
import rospy
import math
import cv2
import os
import rospkg
import tf
from unet_model import *
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from imagine_common.srv import *
from PIL import Image,ImageEnhance
from visualization_msgs.msg import  MarkerArray,Marker
from keras.preprocessing import image
from imagine_common.msg import *
import geometry_msgs.msg
from affordance import affordanceWrapper

class Cut:
    def __init__(self, affordance, required_parts=[]):
        self.affordance=affordance
        self.required_parts_for_affordance=required_parts
        self.model = unetmodel('unet_cut')
        self.bridge = CvBridge()
        self.pixel_world_srv= rospy.ServiceProxy('/perception/pixel2world', ConvertPixel2World)
        self.cut_rviz = rospy.Publisher('asc/cut_points',MarkerArray,queue_size=1)
        self.cut_image_viz = rospy.Publisher('asc/cut_image/compressed',CompressedImage,queue_size=10)

    def sample_cut_points(self,mask,img_shape=256,window_size=6):
        cut_points = list()
        confidence = list()
        offset = 3
        w = window_size//2
        threshold = (window_size**2 - window_size - offset)*255.
        for i in range(w,img_shape-w):
            for j in range(w,img_shape-w):
                if np.sum(mask[i-w:i+w,j-w:j+w])>threshold:
                    cut_points.append((i,j))
                    confidence.append(np.sum(mask[i-w:i+w,j-w:j+w])/(window_size**2)/255.0)
        cut_points_to_send=list()
        confidence_to_send = list()
        for index,point in enumerate(cut_points):
            remove=False
            for point2 in cut_points_to_send:
                if (point[0]-point2[0])**2 + (point[1]-point2[1])**2<8:
                    remove=True
            if remove==False:
                cut_points_to_send.append(point)
                confidence_to_send.append(confidence[index])

        return cut_points_to_send,confidence_to_send

    def find_affordance(self, data):
        aff_list=list()
        except CvBridgeError as e:
            print(e)
        for part in data.part_array:
            partname=part.part_id[:-1]
            if partname=='segment':
                cable=part
        try:
            self.curr_img = self.bridge.imgmsg_to_cv2(cable.part_outline.part_mask, cable.part_outline.part_mask.encoding)
        aff=Affordance()
        aff.object_name=cable.part_id


        img_w = self.curr_img.shape[0]
        img_h = self.curr_img.shape[1]
        aff_mask=self.model.predict(self.curr_img)

        cut_points, confidences=self.sample_cut_points(aff_mask)
        if len(cut_points)==0:
            return aff_list
        
        aff.effect_name='cut'
        aff.affordance_name='cuttable'
        cut_points_converted=ConvertPixel2WorldRequest()

        for i in range(len(cut_points)):
            cut_point = geometry_msgs.msg.Point()
            cut_point.y = cut_points[i][0] * img_w/256.0
            cut_point.x = cut_points[i][1] * img_h/256.0
            cut_point.z = 0
            cut_points_converted.pixels.append(cut_point)
        resp = self.pixel_world_srv(cut_points_converted)
        affordance_vis_image = cv2.resize(aff_mask,(256,256))

        markerArray= MarkerArray()
        for i in range(len(cut_points)):
            marker= Marker()
            ap= ActionParameters()
            ap.confidence=confidences[i]
            cut_point =AscPair()
            cut_point.key = 'cut_pose'
            cut_point.value_type = 2
            cut_point.value_pose.position.x=resp.points[i].x
            cut_point.value_pose.position.y=resp.points[i].y
            cut_point.value_pose.position.z=resp.points[i].z

            cut_point.value_pose.orientation.x=0
            cut_point.value_pose.orientation.y=-0.707
            cut_point.value_pose.orientation.z=0
            cut_point.value_pose.orientation.w=0.707

            ap.parameters.append(cut_point)
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id=resp.header.frame_id
            marker.header=h
            marker.ns= "cut_points"
            marker.id=i
            marker.type= 3 #
            marker.action = 0
            marker.scale.x=0.01
            marker.scale.y=0.001
            marker.scale.z=0.002
            marker.lifetime = rospy.Duration(150)
            marker.color.r=0.0
            marker.color.g=1.0
            marker.color.b=0.0
            marker.color.a=1.0
            marker.pose=cut_point.value_pose   
            markerArray.markers.append(marker)
            aff.action_parameters_array.append(ap)

        aff_list.append(aff)
        rate=rospy.Rate(10)
        comp_img=self.bridge.cv2_to_compressed_imgmsg(affordance_vis_image)
        for _ in range(5):
            self.cut_rviz.publish(markerArray)
            self.cut_image_viz.publish(comp_img)
            rate.sleep()


        return aff_list

