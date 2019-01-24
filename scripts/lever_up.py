#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import os
import rospkg
from unet_model import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imagine_common.srv import *
from imagine_common.msg import *
from geometry_msgs.msg import *
class Lever_Up:
	def __init__(self):
		self.required_parts_for_affordance=['pcb'] 
		self.image_sub = rospy.Subscriber("image", Image, self.image_cb)
		self.curr_img=None
		self.model = unetmodel('leverup')
		self.bridge = CvBridge()
	def image_cb(self,msg):
		try:
			self.curr_img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
		except CvBridgeError as e:
			print("Received Image couldn't make it through the CV Bridge of Death")
	def sample_leverup_points(self,mask,img_shape=256,window_size=6):
		leverup_points = list()
		confidence = list()
		offset = 5
		w = window_size//2
		threshold = (window_size**2 - window_size - offset)*255.
		for i in range(w,img_shape-w):
			for j in range(w,img_shape-w):
				if np.sum(mask[i-w:i+w,j-w:j+w])>threshold:
					leverup_points.append((i,j))
					confidence.append(np.sum(mask[i-w:i+w,j-w:j+w])/(window_size**2)/255*100)
		return leverup_points,confidence
	def find_affordance(self,data):
		aff_list=list()
		any_screw_on_lid=False
		for part in data.part_array:
			partname=part.part_id[:-1]
			if partname=='screw':
				any_screw_on_pcb=True
			any_screw_on_pcb=False 
			if partname=='pcb':
				pcb=part    
		if any_screw_on_pcb==False:
			aff=Affordance()
			aff.object_name=pcb.part_id
			aff_mask=self.model.predict(self.curr_img)
			cv2.imwrite('messigray.png',aff_mask)
			leverup_points,confidences=self.sample_leverup_points(aff_mask)
			print(leverup_points)
			aff.effect_name='levered'
			aff.affordance_name='leverable'
			for i in range(len(leverup_points)):
				ap= ActionParameters()
				ap.confidence=confidences[i]
				lever_up_point =AscPair()
				lever_up_point.key = 'lever_pose'
				lever_up_point.value_type = 2
				lever_up_point.value_pose.position.x=leverup_points[i][0]
				lever_up_point.value_pose.position.y=leverup_points[i][1]
				lever_up_point.value_pose.position.z=600
				ap.parameters.append(lever_up_point)
				aff.action_parameters_array.append(ap)
			aff_list.append(aff)
		return aff_list

