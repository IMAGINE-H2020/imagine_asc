#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import os
import rospkg
import unetmodel
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imagine_common.srv import *
from imagine_common.msg import *
from geometry_msgs.msg import *
class Lever_Up:
    def __init__(self):
        self.required_parts_for_affordance=['pcb'] 
	self.model = unetmodel('leverup')
    def find_affordance(self,data):
	aff_list=[]
        any_screw_on_lid=False
        for part in data.part_array:
            partname=part.part_id[:-1]
            if partname=='screw':
                any_screw_on_pcb=True
            if partname=='pcb':
                pcb=part    
        
        if any_screw_on_pcb==False:
            aff=Affordance()
            aff.object_name=part.part_id
            aff.effect_name='lever_uppable'
            aff.affordance_name='leveruppability'
            ap= ActionParameters()
            ap.confidence=0.95
            asc_pair =AscPair()
	    ## TODO
            ap.parameters.append(asc_pair)
            aff.action_parameters_array.append(ap)
            aff_list.append(aff)
        return aff_list
