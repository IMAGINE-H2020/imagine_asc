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

class affordanceWrapper:
    def __init__(self):
        self.affordance_list=[]
        self.affordance_funcs={}
        self.available_parts=[]
        self.required_parts_for_affordance={}
        self.s = rospy.Service('asc/request_affordances', AffordanceArray, self.requestAffordances)

    def updateDataRequest(self,request):
        self.curr_data=request
        self.available_parts=list()
        for part in self.curr_data.part_array:
            partname=part.part_id[:-1]
            self.available_parts.append(partname)

    def addAffordanceModel(self,object_name,affordance_name,effect_name,required_parts,affordance_func):
        aff=(affordance_name,object_name,effect_name)
        if aff not in self.affordance_list: 
            self.affordance_list.append(aff)
        self.affordance_funcs[aff]=affordance_func
        self.required_parts_for_affordance[aff]=required_parts


    def requestAffordances(self,request):
        self.updateDataRequest(request)
        aff_arr_resp =AffordanceArrayResponse()
        for aff in self.affordance_list:
            if set(self.required_parts_for_affordance[aff]).issubset(self.available_parts):
                aff_array = self.affordance_funcs[aff](self.curr_data)
                for aff in aff_array:
                    aff_arr_resp.affordances.append(aff)
        return aff_arr_resp
