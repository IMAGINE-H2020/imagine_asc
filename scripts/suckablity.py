#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import os
import rospkg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imagine_common.srv import *
from imagine_common.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import MarkerArray,Marker
import numpy as np
from affordance import affordanceWrapper

class Suckability:
    def __init__(self,affordance,required_parts=['lid']):
        self.affordance=affordance
        self.required_parts_for_affordance=required_parts
        self.suck_rviz = rospy.Publisher('asc/suck_points',MarkerArray,queue_size=1)
        self.bridge = CvBridge()
        self.pixel_world_srv= rospy.ServiceProxy('/perception/pixel2world', ConvertPixel2World)

    def image_resize(self, image, width = 256,height = 256):
        return cv2.resize(image, (width, height), interpolation = cv2.INTER_AREA)

    def findCOM(self, img, width = 256,height = 256):
        m = np.zeros((width, height))
        for x in range(width):
            for y in range(height):
                if img[x,y] != 0:
                    m[x, y] = 1
                else:
                    m[x, y] = 0
        m = m / np.sum(np.sum(m))
        dx = np.sum(m, 1)
        dy = np.sum(m, 0)
        # expected values
        cx = np.sum(dx * np.arange(width))
        cy = np.sum(dy * np.arange(height))

        return np.array([cx,cy])
    
    def findSuckPoints(self, img, width = 256, height = 256, s_width = 40, s_height = 30):
        suck_points = []
        mask_center = self.findCOM(img,width,height)
        scale_min = 0.2
        scale_range = 1 - scale_min
        for x in range(width-s_width):
            for y in range(height-s_height):
                if np.argwhere(img[x:x+s_width, y:y+s_height] == 0).size == 0:
                    suction_candidate = np.array([x+s_width//2, y+s_height//2])
                    distance_from_center = np.linalg.norm(mask_center - suction_candidate)
                    confidence = scale_range - (distance_from_center / 361.0 * scale_range) + scale_min 
                    suck_points.append([suction_candidate[0], suction_candidate[1], confidence])
        #suck_points_to_send=list()
        #for point in suck_points:
        #    remove=False
        #    for point2 in suck_points_to_send:
        #        if (point[0]-point2[0])**2 + (point[1]-point2[1])**2<8:
        #            remove=True
        #    if remove==False:
        #        suck_points_to_send.append(point)
        
        return np.array(suck_points)[np.random.permutation(len(suck_points))[:50]]

    def find_affordance(self,data):
        aff_list=[]
        markerArray= MarkerArray()
        no = 0    
        any_screw_on_lid=False
        subtract_flag=False
        for part_ in data.part_array:
            partname=part_.part_id[:-1]
            if partname==self.affordance[0]:
                part=part_
            if partname=='platters_clamp':
                subtract_flag=True
                subtract_part=part_
        part_mask = self.bridge.imgmsg_to_cv2(part.part_outline.part_mask, part.part_outline.part_mask.encoding)
        img_w, img_h = part_mask.shape[0], part_mask.shape[1]
        width_scaled = 256
        height_scaled = 256
        part_mask_scaled=self.image_resize(part_mask,width_scaled,height_scaled)

        if self.affordance[0]=='magnet':
            suck_points = self.findSuckPoints(part_mask_scaled, width_scaled, height_scaled, 20, 20)
        elif self.affordance[0]=='lid' or self.affordance[0]=='pcb':
            suck_points = self.findSuckPoints(part_mask_scaled, width_scaled, height_scaled, 40, 30)
        elif self.affordance[0]=='platter':
            if subtract_flag:
                subtract_part_mask = self.bridge.imgmsg_to_cv2(subtract_part.part_outline.part_mask, subtract_part.part_outline.part_mask.encoding)
                subtract_part_mask_scaled=self.image_resize(subtract_part_mask,256,256)
                part_mask_scaled=part_mask_scaled-subtract_part_mask_scaled
            suck_points = self.findSuckPoints(part_mask_scaled, width_scaled, height_scaled, 40, 30)
        else:
            return aff_list
        if len(suck_points)==0:
            return aff_list
            
        suck_points_converted=ConvertPixel2WorldRequest()
        for i in range(len(suck_points)):
            suck_point = geometry_msgs.msg.Point()
            suck_point.y = suck_points[i,0] * img_w/256.0 #todo
            suck_point.x = suck_points[i,1] * img_h/256.0
            suck_point.z = 0
            suck_points_converted.pixels.append(suck_point)

            point_inverted = np.array(suck_points[i,::-1])[1:]
            cv2.circle(affordanceWrapper.affordance_vis_image, tuple(point_inverted.astype(np.int16)) , 3, (0, 255, 255), -1)
        resp = self.pixel_world_srv(suck_points_converted)
        aff=Affordance()
        aff.object_name=part.part_id
        aff.effect_name='suckable'
        aff.affordance_name='suckability'
        markerArray= MarkerArray()
        for i in range(len(suck_points)):
            marker= Marker()
            ap= ActionParameters()
            ap.confidence=suck_points[i,2]
            suck_point =AscPair()
            suck_point.key = 'start_pose'
            suck_point.value_type = 2
            suck_point.value_pose.position.x=resp.points[i].x
            suck_point.value_pose.position.y=resp.points[i].y
            suck_point.value_pose.position.z=resp.points[i].z
            suck_point.value_pose.orientation.x=0
            suck_point.value_pose.orientation.y=-0.707
            suck_point.value_pose.orientation.z=0
            suck_point.value_pose.orientation.w=0.707

            ap.parameters.append(suck_point)
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id=resp.header.frame_id
            marker.header=h
            marker.ns= "suck_point_"+part.part_id
            marker.id=i
            marker.type= 3 
            marker.action = 0
            marker.scale.x=0.01
            marker.scale.y=0.001
            marker.scale.z=0.002
            marker.lifetime = rospy.Duration(150)
            marker.color.r=1.0
            marker.color.g=1.0
            marker.color.b=0.0
            marker.color.a=1.0
            marker.pose=suck_point.value_pose   
            markerArray.markers.append(marker)
            aff.action_parameters_array.append(ap)
        aff_list.append(aff)
        rate=rospy.Rate(10)
        for _ in range(5):
            self.suck_rviz.publish(markerArray)
            rate.sleep()
        return aff_list
