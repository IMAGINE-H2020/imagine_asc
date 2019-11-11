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
    screwability=Screwability(('screw','unscrewability','unscrewed'))
    affordance_wrapper.addAffordanceModel(screwability)

    lid_suckablity=Suckability('lid','suckability','suckable',['lid'])
    magnet_suckablity=Suckability('magnet','suckability','suckable',['magnet'])
    platter_suckablity=Suckability('platter','suckability','suckable',['platter'])

    affordance_wrapper.addAffordanceModel(lid_suckablity)
    affordance_wrapper.addAffordanceModel(magnet_suckablity)
    affordance_wrapper.addAffordanceModel(platter_suckablity)

    leverability=Lever_Up(('pcb','leverability','leverable'),['pcb'])
    affordance_wrapper.addAffordanceModel(leverability)

    ##########################################################
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
