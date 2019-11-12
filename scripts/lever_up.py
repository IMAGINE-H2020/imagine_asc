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

class Lever_Up:
    def __init__(self,affordance,required_parts=[]):
        self.affordance=affordance
        self.required_parts_for_affordance=required_parts
        self.model = unetmodel('unet_lever_up')
        self.bridge = CvBridge()
        self.pixel_world_srv= rospy.ServiceProxy('/perception/pixel2world', ConvertPixel2World)
        self.lever_up_rviz = rospy.Publisher('asc/lever_up_points',MarkerArray,queue_size=1)
        self.lever_up_image_viz = rospy.Publisher('asc/lever_up_image/compressed',CompressedImage,queue_size=10)

    def sample_leverup_points(self,mask,img_shape=256,window_size=6):
        leverup_points = list()
        confidence = list()
        offset = 3
        w = window_size//2
        threshold = (window_size**2 - window_size - offset)*255.
        for i in range(w,img_shape-w):
            for j in range(w,img_shape-w):
                if np.sum(mask[i-w:i+w,j-w:j+w])>threshold:
                    leverup_points.append((i,j))
                    confidence.append(np.sum(mask[i-w:i+w,j-w:j+w])/(window_size**2)/255.0)
        leverup_points_to_send=list()
        confidence_to_send = list()
        for index,point in enumerate(leverup_points):
            remove=False
            for point2 in leverup_points_to_send:
                if (point[0]-point2[0])**2 + (point[1]-point2[1])**2<8:
                    remove=True
            if remove==False:
                leverup_points_to_send.append(point)
                confidence_to_send.append(confidence[index])

        return leverup_points_to_send,confidence_to_send

    def find_affordance(self,data):
        aff_list=list()
        try:
            tmp_img = self.bridge.imgmsg_to_cv2(data.assos_img, 'rgb8')
            tmp_img2=ImageEnhance.Color(Image.fromarray(tmp_img)).enhance(2)
            self.curr_img=image.img_to_array(tmp_img2)
        except CvBridgeError as e:
            print(e)
        for part in data.part_array:
            partname=part.part_id[:-1]
            if partname=='pcb':
                pcb=part   
        aff=Affordance()
        aff.object_name=pcb.part_id
        img_w = self.curr_img.shape[0]
        img_h = self.curr_img.shape[1]
        print img_w,img_h 
        aff_mask=self.model.predict(self.curr_img)

        leverup_points,confidences=self.sample_leverup_points(aff_mask)
        aff.effect_name='levered'
        aff.affordance_name='leverable'
        leverup_points_converted=ConvertPixel2WorldRequest()
        for i in range(len(leverup_points)):
            lever_up_point = geometry_msgs.msg.Point()
            lever_up_point.y = leverup_points[i][0] * img_w/256.0
            lever_up_point.x = leverup_points[i][1] * img_h/256.0
            lever_up_point.z = 0
            leverup_points_converted.pixels.append(lever_up_point)
        resp = self.pixel_world_srv(leverup_points_converted)
        affordance_vis_image = cv2.resize(tmp_img,(256,256))
        tmp_img = self.bridge.imgmsg_to_cv2(pcb.part_outline.part_mask, pcb.part_outline.part_mask.encoding)
        tmp_img = cv2.resize(tmp_img, (256, 256))
        markerArray= MarkerArray()
        for i in range(len(leverup_points)):
            marker= Marker()
            ap= ActionParameters()
            ap.confidence=confidences[i]
            lever_up_point =AscPair()
            lever_up_point.key = 'start_pose'
            lever_up_point.value_type = 2
            lever_up_point.value_pose.position.x=resp.points[i].x
            lever_up_point.value_pose.position.y=resp.points[i].y
            lever_up_point.value_pose.position.z=resp.points[i].z
            x= leverup_points[i][0]
            y= leverup_points[i][1]
            w_size=9
            temp=tmp_img[max(0,x-w_size):min(256,x+w_size),max(0,y-w_size):min(256,y+w_size)]
            x1,y1= np.where(temp==255)
            x2,y2= np.where(temp==0)
            direction =math.pi + math.atan2(np.mean(y2)-np.mean(y1),np.mean(x2)-np.mean(x1))
            if len(y2)==0 or len(y1)==0 or len(x2)==0 or len(x1)==0 or np.isnan(direction) or np.isinf(direction):
                continue
            import tf
            point_inverted = np.array(leverup_points[i][::-1])
            _ = cv2.arrowedLine(affordance_vis_image, tuple(point_inverted.astype(np.int16)), tuple((point_inverted+[np.sin(direction)*15,np.cos(direction)*15]).astype(np.int16)), (255,0,0), 1, tipLength=0.5)
            quaternion = tf.transformations.quaternion_from_euler(0,0,direction)
            lever_up_point.value_pose.orientation.x=quaternion[0]
            lever_up_point.value_pose.orientation.y=quaternion[1]
            lever_up_point.value_pose.orientation.z=quaternion[2]
            lever_up_point.value_pose.orientation.w=quaternion[3]

            ap.parameters.append(lever_up_point)
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id=resp.header.frame_id
            marker.header=h
            marker.ns= "lever_up_points"
            marker.id=i
            marker.type= 0 # arrow
            marker.action = 0
            marker.scale.x=0.01
            marker.scale.y=0.001
            marker.scale.z=0.002
            marker.lifetime = rospy.Duration(90)
            marker.color.r=1.0
            marker.color.g=0.0
            marker.color.b=0.0
            marker.color.a=1.0
            marker.pose=lever_up_point.value_pose   
            markerArray.markers.append(marker)
            aff.action_parameters_array.append(ap)
        aff_list.append(aff)
        rate=rospy.Rate(10)
        comp_img=self.bridge.cv2_to_compressed_imgmsg(affordance_vis_image)
        for _ in range(5):
            self.lever_up_rviz.publish(markerArray)
            self.lever_up_image_viz.publish(comp_img)
            rate.sleep()


        return aff_list

