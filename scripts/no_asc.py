#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import os
import rospkg
from affordance import affordanceWrapper
from screwability import *
def main(args):
    rospy.init_node('asc')
    affordance_wrapper = affordanceWrapper()
    ######### Add affordance models here #####################
    screwability=Screwability(('screw','unscrewability','unscrewed'))
    affordance_wrapper.addAffordanceModel(screwability)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
