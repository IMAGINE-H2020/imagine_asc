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
        res = effect_marker(pose,'unscrew','pcb1')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    pose = Pose()
    pose.position.x = 0.599419863645
    pose.position.y = -0.0968418264053
    pose.position.z = 0.127468142134
    r = tf.transformations.quaternion_from_euler(0, 0, (-90/180.)*math.pi)
    pose.orientation.x = -0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.735224174106
    pose.orientation.w = -0.677824028646
    print "Requesting ",pose
    print effect_marker_client(pose)
