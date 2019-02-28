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
from visualization_msgs.msg import	MarkerArray,Marker
from keras.preprocessing import image
from imagine_common.msg import *
import geometry_msgs.msg
class Lever_Up:
	def __init__(self):
		self.required_parts_for_affordance=[]#['pcb'] 
		self.model = unetmodel('unet_lever_up')
		self.bridge = CvBridge()
		self.pixel_world_srv= rospy.ServiceProxy('perception/pixel2world', ConvertPixel2World)
		self.lever_up_rviz = rospy.Publisher('asc/lever_up_points',MarkerArray,queue_size=1)

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
#			import IPython; IPython.embed()
			tmp_img = self.bridge.imgmsg_to_cv2(data.assos_img, data.assos_img.encoding)
			tmp_img_ = np.zeros_like(tmp_img)
			tmp_img_[:,:,0]=tmp_img[:,:,2]
			tmp_img_[:,:,1]=tmp_img[:,:,1]
			tmp_img_[:,:,2]=tmp_img[:,:,0]
			#tmp_img2=ImageEnhance.Color(Image.fromarray(tmp_img_)).enhance(4)
			self.curr_img=tmp_img_#image.img_to_array(tmp_img2)		
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
			print img_w,img_h 
			aff_mask=self.model.predict(self.curr_img)

			cv2.imwrite('/home/colors/messi.png',aff_mask)
			leverup_points,confidences=self.sample_leverup_points(aff_mask)
			aff.effect_name='levered'
			aff.affordance_name='leverable'
			leverup_points_converted=ConvertPixel2WorldRequest()
			for i in range(len(leverup_points)):
				lever_up_point = geometry_msgs.msg.Point()
				lever_up_point.y = leverup_points[i][0] * img_w/256.0
				lever_up_point.x = leverup_points[i][1] * img_h/256.0
				lever_up_point.z = 0
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
				lever_up_point.value_pose.position.z=resp.points[i].z
				if i==0:
					print(lever_up_point)
				ind= np.random.randint(0,len(leverup_points))
				x= leverup_points[ind][0]
				y= leverup_points[ind][1]
				w_size=9
				tmp_img = self.bridge.imgmsg_to_cv2(pcb.part_outline.part_mask, pcb.part_outline.part_mask.encoding)
				tmp_img = cv2.resize(tmp_img, (256, 256)) 
				temp=tmp_img[max(0,x-w_size):min(256,x+w_size),max(0,y-w_size):min(256,y+w_size)]

				x1,y1= np.where(temp==255)
				x2,y2= np.where(temp==0)
				direction =math.pi/2 +math.atan2(np.mean(y2)-np.mean(y1),np.mean(x2)-np.mean(x1))
				if np.isnan(direction) or np.isinf(direction):
					continue
				import tf
				quaternion = tf.transformations.quaternion_from_euler(0,0,direction)
				lever_up_point.value_pose.orientation.x=quaternion[0]
				lever_up_point.value_pose.orientation.y=quaternion[1]
				lever_up_point.value_pose.orientation.z=quaternion[2]
				lever_up_point.value_pose.orientation.w=quaternion[3]

				ap.parameters.append(lever_up_point)
				h = std_msgs.msg.Header()
				h.stamp = rospy.Time.now()
				h.frame_id=resp.header.frame_id#"world"#"lever_up_pose"
				marker.header=h
				marker.ns= "lever_up_points"
				marker.id=i
				marker.type= 0 # arrow
				marker.action = 0
				marker.scale.x=0.01
				marker.scale.y=0.005
				marker.scale.z=0.001
				marker.lifetime = rospy.Duration(0)
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

