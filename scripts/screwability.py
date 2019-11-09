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

class Screwability:
    def __init__(self):
        self.required_parts_for_affordance=[] 
	self.unscrew_rviz = rospy.Publisher('asc/unscrew_points',MarkerArray,queue_size=1)
    def find_affordance(self,data):
        aff_list=[]
	markerArray= MarkerArray()
	no = 0	
        for part in data.part_array:
	    partname=part.part_id[:-1]
	    marker= Marker()
            if partname=='screw' or partname=='bearing':
                aff=Affordance()
                aff.object_name=part.part_id
                aff.effect_name='unscrewable'
                aff.affordance_name='unscrewability'
                ap= ActionParameters()
                if (partname == 'bearing'):
                    ap.confidence = part.part_type_specifics_confidence  
                else:
                    ap.confidence = part.part_type_confidence                                      
                asc_pair =AscPair()
                asc_pair.key='start_pose'
                asc_pair.value_type=2
                asc_pair.value_pose=part.pose
                ap.parameters.append(asc_pair)
                aff.action_parameters_array.append(ap)
                aff_list.append(aff)
		
		#marker here		
		h = std_msgs.msg.Header()
		h.stamp = rospy.Time.now()
		h.frame_id="world" #.header.frame_id #"world"#"unscrew_pose"
		marker.header=h
		marker.ns= "unscrew_points"
		no = no + 1
		marker.id = no #int(markerid)
		marker.type= 3 # cylinder
		marker.action = 0
		marker.scale.x=0.005
		marker.scale.y=0.005
		marker.scale.z=0.005
		marker.lifetime = rospy.Duration(0)
		marker.color.r=0.0
		marker.color.g=0.0
		marker.color.b=1.0
		marker.color.a=1.0
		marker.pose=part.pose
		markerArray.markers.append(marker)
		aff.action_parameters_array.append(ap)
		
	rate=rospy.Rate(10)
	for _ in range(5):
		self.unscrew_rviz.publish(markerArray)
		rate.sleep()
        return aff_list
