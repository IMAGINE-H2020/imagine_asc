#!/usr/bin/env python
import roslib
import sys
import rospy
import math
import cv2
import os
import rospkg
import tf
from unet_model import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from imagine_common.srv import *
from PIL import Image,ImageEnhance
from visualization_msgs.msg import	MarkerArray
from keras.preprocessing import image
from imagine_common.msg import *
from geometry_msgs.msg import *
class Lever_Up:
	def __init__(self):
		self.required_parts_for_affordance=['pcb'] 
		self.model = unetmodel('unet_lever_up')
		self.bridge = CvBridge()
		self.pixel_world_srv= rospy.ServiceProxy("/perception/world2pixel", ConvertPixel2World	)
		self.lever_up_rviz = rospy.Publisher('/asc/lever_up_points',MarkerArray)

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
		any_screw_on_pcb=False
		try:
			tmp_img = self.bridge.imgmsg_to_cv2(data.assos_img, "bgr8")
			tmp_img2=ImageEnhance.Color(tmp_img).enhance(4)
			self.curr_img=image.img_to_array(tmp_img2)		
		except CvBridgeError as e:
			print(e)
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
			img_w = self.curr_img.shape[0]
			img_h = self.curr_img.shape[1]
			aff_mask=self.model.predict(self.curr_img)
			cv2.imwrite('messi.png',aff_mask)
			leverup_points,confidences=self.sample_leverup_points(aff_mask)
			aff.effect_name='levered'
			aff.affordance_name='leverable'
			leverup_points_converted=ConvertPixel2WorldRequest()
			for i in range(len(leverup_points)):
				lever_up_point = Point()
				lever_up_point.x = leverup_points[i][0] * img_w/256.0
				lever_up_point.y = leverup_points[i][1] * img_h/256.0
				leverup_points_converted.pixels.append(lever_up_point)
			resp = self.pixel_world_srv(leverup_points_converted)
			markerArray= MarkerArray()
			for i in range(len(leverup_points)):
				marker= Marker()
				ap= ActionParameters()
				ap.confidence=confidences[i]
				lever_up_point =AscPair()
				lever_up_point.key = 'lever_pose'
				lever_up_point.value_type = 2
				lever_up_point.value_pose.position.x=resp.points[i].x
				lever_up_point.value_pose.position.y=resp.points[i].y
				lever_up_point.value_pose.position.x=resp.points[i].z
				ind= np.random.randint(0,len(leverup_points))
				x= leverup_points[ind][0]
				y= leverup_points[ind][1]
				w_size=9
				temp=img[max(0,x-w_size):min(256,x+w_size),max(0,y-w_size):min(256,y+w_size),:]
				hsv_temp = cv2.cvtColor(temp, cv2.COLOR_RGB2HSV)
				green_start=(65, 0, 40)
				green_end=(150, 255, 255)
				mask = cv2.inRange(hsv_temp, green_start, green_end)
				x1,y1= np.where(mask==255)
				x2,y2= np.where(mask==0)
				direction =math.atan2(y2-y1,x2-x1)
				quaternion = tf.transformations.quaternion_from_euler(0, 0, direction)
				lever_up_point.value_pose.orientation=quaternion
				ap.parameters.append(lever_up_point)
				h = std_msgs.msg.Header()
				h.stamp = rospy.Time.now()
				h.frame_id="world"#"lever_up_pose"
				marker.header=h
				marker.ns= "lever_up_points"
				marker.id=i
				marker.type= 0 # arrow
				marker.action = 0
				marker.scale.x=0.01
				marker.scale.y=0.01
				marker.scale.z=0.01
				marker.lifetime = rospy.Duration(5000)
				marker.color.r=1.0
				marker.color.g=0.0
				marker.color.b=0.0
				marker.color.a=1.0
				marker.pose=lever_up_point.value_pose
				markerArray.markers.append(marker)
				aff.action_parameters_array.append(ap)
			aff_list.append(aff)
			self.lever_up_rviz.publish(markerArray)
		return aff_list

