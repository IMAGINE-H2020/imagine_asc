#!/usr/bin/env python
import sys
import rospy
from imagine_common.srv import AffordanceArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
def get_affordance_client():

  rospy.wait_for_service('/asc/request_affordances')
  try:
    serv = rospy.ServiceProxy('/asc/request_affordances', AffordanceArray)
    resp = serv()
    return resp
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e


def main(args):
  rospy.init_node('affordance_test', anonymous=False)
  resp=get_affordance_client()
  print resp


if __name__ == '__main__':
  main(sys.argv)
