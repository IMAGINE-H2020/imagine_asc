#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped
from imagine_common.srv import EffectMarker
from imagine_common.srv import EffectMarkerResponse
from visualization_msgs.msg import Marker
import std_msgs.msg
import tf
import numpy as np
import rospkg
import sys

effect_publisher = None
marker = None

def handle_effect_marker(req):
    global effect_publisher, marker
    print "Got a Request."
    marker.pose.position.x = req.pose.position.x #0.7
    marker.pose.position.y = req.pose.position.y #0.4
    marker.pose.position.z = req.pose.position.z #0.6

    marker.pose.orientation.x = req.pose.orientation.x #0
    marker.pose.orientation.y = req.pose.orientation.y #0
    marker.pose.orientation.z = req.pose.orientation.z #0
    marker.pose.orientation.w = req.pose.orientation.w #1.0
    rate = rospy.Rate(100)
    for i in range(100):
        print marker
        effect_publisher.publish(marker)
        rate.sleep()
    return EffectMarkerResponse(True)

def effect_marker_server():
    global effect_publisher,marker
    rospy.init_node('effect_marker_server')
    s = rospy.Service('effect_marker', EffectMarker, handle_effect_marker)
    effect_publisher = rospy.Publisher('/asc/lever_up_effect', Marker, queue_size=10)
    rospack = rospkg.RosPack()

    effect = np.loadtxt(rospack.get_path('imagine_asc')+"/10_10.txt")/2.
    marker= Marker()
    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    h.frame_id="world"#"lever_up_pose"
    marker.header=h
    marker.ns= "lever_up_effect"
    marker.id=0
    marker.type=4 # line strip
    marker.action = 0
    marker.scale.x=0.01
    marker.scale.y=0.01
    marker.scale.z=0.01
    marker.lifetime = rospy.Duration(0)
    marker.color.r=0.0
    marker.color.g=1.0
    marker.color.b=0.0
    marker.color.a=1.0
    for i in range(effect.shape[0]): 
        point = Point()
        point.x=effect[i,1]
        point.y=effect[i,0]
        point.z=effect[i,2]-0.325
        marker.points.append(point)
    print "Effect Marker is ready."
    rospy.spin()

if __name__ == '__main__':
    effect_marker_server()
    rospy.spin()

