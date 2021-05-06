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
from cut import *
from lever_up import *
from Push import *
def main(args):
    rospy.init_node('asc')
    affordance_wrapper = affordanceWrapper()
    ######### Add affordance models here #####################
    screwability=Screwability(('screw','unscrewability','unscrewed'))
    affordance_wrapper.addAffordanceModel(screwability)

    lid_suckablity=Suckability(('lid','suckability','suckable'),['lid'])
    magnet_suckablity=Suckability(('magnet','suckability','suckable'),['magnet'])
    platter_suckablity=Suckability(('platter','suckability','suckable'),['platter'])
    pcb_suckablity=Suckability(('pcb','suckability','suckable'),['pcb'])

    affordance_wrapper.addAffordanceModel(lid_suckablity)
    affordance_wrapper.addAffordanceModel(pcb_suckablity)
    affordance_wrapper.addAffordanceModel(magnet_suckablity)
    affordance_wrapper.addAffordanceModel(platter_suckablity)

    pcb_leverability=Lever_Up(('pcb','leverability','leverable'),['pcb'])
    magnet_leverability=Lever_Up(('magnet','leverability','leverable'),['magnet'])
    affordance_wrapper.addAffordanceModel(pcb_leverability)
    affordance_wrapper.addAffordanceModel(magnet_leverability)

    cuttability=Cut(('segment','cuttability','cuttable'),['segment'])
    affordance_wrapper.addAffordanceModel(cuttability)

    pushablity = Push(("spindle_hub", 'pushability', 'pushable'),['spindle_hub'])
    affordance_wrapper.addAffordanceModel(pushablity)

    ##########################################################
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)
