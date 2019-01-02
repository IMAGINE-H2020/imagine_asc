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
class Screwability:
    def __init__(self):
        self.required_parts_for_affordance=['screw'] 
    def find_affordance(self,data):
        aff_list=[]
        for part in data.part_array:
            partname=part.part_id[:-1]
            if partname=='screw':
                aff=Affordance()
                aff.object_name=part.part_id
                aff.effect_name='unscrewable'
                aff.affordance_name='unscrewability'
                ap= ActionParameters()
                ap.confidence=0.95
                asc_pair =AscPair()
                asc_pair.key='start position'
                asc_pair.value_type=2
                asc_pair.value_pose=part.pose
                ap.parameters.append(asc_pair)
                aff.action_parameters_array.append(ap)
                aff_list.append(aff)
        return aff_list