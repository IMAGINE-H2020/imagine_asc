#!/usr/bin/env python
import sys
import rospy
import cv2
from imagine_common.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

def main(args):
  rospy.init_node('affordance_test', anonymous=False)
  print('here1')
  rospy.wait_for_service('asc/request_affordances')
  serv = rospy.ServiceProxy('asc/request_affordances', AffordanceArray)
  print('here2')
  req=AffordanceArrayRequest()
  img = cv2.imread('image3.png',1)
  bridge = CvBridge()
  req.assos_img=bridge.cv2_to_imgmsg(img)  
  resp = serv(req)
  print resp


if __name__ == '__main__':
  main(sys.argv)
