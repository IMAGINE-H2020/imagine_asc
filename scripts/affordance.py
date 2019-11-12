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
from sensor_msgs.msg import Image, CompressedImage


class affordanceWrapper:
    comp_img = []
    affordance_vis_image = []

    def __init__(self):
        self.affordance_list = []
        self.affordance_funcs = {}
        self.available_parts = []
        self.required_parts_for_affordance = {}
        self.s = rospy.Service('asc/request_affordances2', AffordanceArray, self.requestAffordances)
        self.affordances_image_rviz = rospy.Publisher('asc/debug/affordances_image/compressed', CompressedImage, queue_size=10)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

    def updateDataRequest(self, request):
        self.curr_data = request
        self.available_parts = list()
        tmp_img = self.bridge.imgmsg_to_cv2(self.curr_data.assos_img, self.curr_data.assos_img.encoding)
        affordanceWrapper.affordance_vis_image = cv2.resize(tmp_img, (256, 256))


        for part in self.curr_data.part_array:
            partname = part.part_id[:-1]
            self.available_parts.append(partname)

    def addAffordanceModel(self, affordance):
        aff = affordance.affordance
        if aff not in self.affordance_list:
            self.affordance_list.append(aff)
        self.affordance_funcs[aff] = affordance.find_affordance
        self.required_parts_for_affordance[aff] = affordance.required_parts_for_affordance

    def requestAffordances(self, request):
        self.updateDataRequest(request)
        aff_arr_resp = AffordanceArrayResponse()
        for aff in self.affordance_list:
            if set(self.required_parts_for_affordance[aff]).issubset(self.available_parts):
                aff_array = self.affordance_funcs[aff](self.curr_data)
                for aff in aff_array:
                    aff_arr_resp.affordances.append(aff)

        affordanceWrapper.comp_img = self.bridge.cv2_to_compressed_imgmsg(affordanceWrapper.affordance_vis_image)

        for _ in range(5):
            self.affordances_image_rviz.publish(affordanceWrapper.comp_img)
            self.rate.sleep()

        return aff_arr_resp
