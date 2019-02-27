#!/usr/bin/env python

import sys
import math
import rospy
from imagine_common.srv import EffectMarker
from geometry_msgs.msg import Pose
import tf 

def effect_marker_client(pose):
    rospy.wait_for_service('effect_marker')
    try:
        effect_marker = rospy.ServiceProxy('effect_marker', EffectMarker)
        res = effect_marker(pose)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    pose = Pose()
    pose.position.x = 0.559
    pose.position.y = -0.086
    pose.position.z = 0.002
    r = tf.transformations.quaternion_from_euler(0, 0, (-90/180.)*math.pi)
    pose.orientation.x = r[0]
    pose.orientation.y = r[1]
    pose.orientation.z = r[2]
    pose.orientation.w = r[3]
    print "Requesting ",pose
    print effect_marker_client(pose)