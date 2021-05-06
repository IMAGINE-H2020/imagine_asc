#!/usr/bin/env python
import roslib
import sys
import rospy
import math
import os
import rospkg
import tf
from imagine_common.srv import *
from imagine_common.msg import *
import geometry_msgs.msg
from affordance import affordanceWrapper

class Push:
    def __init__(self,affordance,required_parts=[]):
        self.affordance=affordance
        self.required_parts_for_affordance = required_parts

    def find_affordance(self,data):
        aff_list=list()
        for part in data.part_array:
            partname = part.part_id[:-1]
            if partname == 'spindle_hub':
                aff = Affordance()
                aff.object_name = part.part_id
                aff.effect_name = 'pushed'
                aff.affordance_name = 'pushable'

                ap= ActionParameters()
                ap.confidence =  0.8 # Dummy Value                                     
                asc_pair = AscPair()
                asc_pair.key = 'start_pose'
                asc_pair.value_type = 2
                asc_pair.value_pose = part.pose
                ap.parameters.append(asc_pair)
                aff.action_parameters_array.append(ap)
                aff_list.append(aff)
        return aff_list

