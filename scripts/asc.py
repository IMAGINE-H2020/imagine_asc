#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import os
import rospkg
from affordance import affordanceWrapper
from screwability import *
from suckablity import *
from lever_up import *
def main(args):
  rospy.init_node('asc')
  affordance_wrapper = affordanceWrapper()
  ######### Add affordance models here #####################
  screwability=Screwability()
  affordance_wrapper.addAffordanceModel('screw','unscrewability','unscrewed',screwability.required_parts_for_affordance,screwability.find_affordance)
  suckablity=Suckability()
  affordance_wrapper.addAffordanceModel('lid','suckability','suckable',suckablity.required_parts_for_affordance,suckablity.find_affordance)
  leverability=Lever_Up()
  affordance_wrapper.addAffordanceModel('pcb','leverability','leverable',leverability.required_parts_for_affordance,leverability.find_affordance)
  ##########################################################
  rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
