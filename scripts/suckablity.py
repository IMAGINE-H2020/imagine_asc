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

class Suckability:
    def __init__(self):
        self.required_parts_for_affordance=['lid'] 
	self.suck_rviz = rospy.Publisher('asc/suck_points',MarkerArray,queue_size=1, latch=True)

    def find_affordance(self,data):
        aff_list=[]
	markerArray= MarkerArray()
	no = 0	
        any_screw_on_lid=False
        for part in data.part_array:
            partname=part.part_id[:-1]
            if partname=='screw':
                any_screw_on_lid=True
            if partname=='lid':
                lid=part    
        any_screw_on_lid=False
        if any_screw_on_lid==False:
	    marker= Marker()
            aff=Affordance()
            aff.object_name=lid.part_id
            aff.effect_name='suckable'
            aff.affordance_name='suckability'
            ap= ActionParameters()
            ap.confidence=0.95
            asc_pair =AscPair()
            asc_pair.key='start_pose'
            asc_pair.value_type=2
            asc_pair.value_pose=lid.pose
            ap.parameters.append(asc_pair)
	    #marker here		
	    h = std_msgs.msg.Header()
	    h.stamp = rospy.Time.now()
	    h.frame_id="world" #.header.frame_id #"world"#"unscrew_pose"
	    marker.header=h
	    marker.ns= "suck_points"
	    no = no + 1
	    marker.id = no #int(markerid)
	    marker.type= 0 # cylinder
	    marker.action = 0
	    marker.scale.x=0.03
	    marker.scale.y=0.005
	    marker.scale.z=0.005
	    marker.lifetime = rospy.Duration(0)
	    marker.color.r=1.0
	    marker.color.g=1.0
	    marker.color.b=0.0
	    marker.color.a=1.0
	    marker.pose=lid.pose
	    marker.pose.orientation.x=0
	    marker.pose.orientation.y=-0.707
	    marker.pose.orientation.z=0
	    marker.pose.orientation.w=0.707
	    markerArray.markers.append(marker)
            aff.action_parameters_array.append(ap)
            aff_list.append(aff)
    	rate=rospy.Rate(10)
	for _ in range(20):
		self.suck_rviz.publish(markerArray)
		rate.sleep()
        return aff_list
