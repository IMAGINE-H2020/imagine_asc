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
import pickle
effect_publisher = None
rospack = rospkg.RosPack()

# TODO: Test this.
from lever_up_pn.lever_up_effect import Lever_Up_Effect
lev_up = Lever_Up_Effect()
#with open(rospack.get_path('imagine_asc') +'/predicted_effects.pickle', 'rb') as handle:
#    lever_up_effects = pickle.load(handle)

def handle_effect_marker(req):
    global effect_publisher,lever_up_effects
    print "Got a Request."
    request_name = req.action_name

    if request_name not in ['lever','suck','unscrew']:
        return EffectMarkerResponse(False)
    if request_name == 'lever' and req.part_id[:-1] !='pcb':
        return EffectMarkerResponse(False)
    marker = Marker()
    marker.pose.position.x = req.pose.position.x  # 0.7
    marker.pose.position.y = req.pose.position.y  # 0.4
    marker.pose.position.z = req.pose.position.z  # 0.6

    marker.pose.orientation.x = req.pose.orientation.x  # 0
    marker.pose.orientation.y = req.pose.orientation.y  # 0
    marker.pose.orientation.z = req.pose.orientation.z  # 0
    marker.pose.orientation.w = req.pose.orientation.w  # 1.0

    h = std_msgs.msg.Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "world"  # "lever_up_pose"
    marker.header = h
    marker.id = 0
    marker.type = 4  # line strip
    marker.action = 0
    marker.scale.x = 0.005
    marker.scale.y = 0.005
    marker.scale.z = 0.005
    marker.lifetime = rospy.Duration(150)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    if request_name == 'lever':

        marker.ns = "lever_up_effect"
        direction_theta=tf.transformations.euler_from_quaternion([marker.pose.orientation.x,marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w])[2]/np.pi*180
        if np.abs(direction_theta + 90) <= 45:
            direction=0
        elif direction_theta + 90 < -45:
            direction=-np.pi/2
        elif direction_theta + 90 > 45:
            direction=np.pi/2
        quaternion = tf.transformations.quaternion_from_euler(0, 0, direction)

        marker.pose.orientation.x = quaternion[0]  # 0
        marker.pose.orientation.y = quaternion[1]  # 0
        marker.pose.orientation.z = quaternion[2]  # 0
        marker.pose.orientation.w = quaternion[3]  # 1.0

        pcb_bounding_values = rospy.get_param('pcb_bounding_values')
        pcb_size_x = pcb_bounding_values[0]  ## ROSPARAM
        pcb_size_y = pcb_bounding_values[1]  ## ROSPARAM

        #selection_criterias=('no-wall',max(0,min(9,int((pcb_size_y-0.03)/0.005))))

        #if np.abs(direction_theta+90) >45:
        #    selection_criterias=('wall',max(0,min(6,int((pcb_size_x-0.05)/0.005))))

        effect = lev_up.get_pcb_trajectory(pcb_size_x,pcb_size_y,direction_theta) #lever_up_effects[selection_criterias]
        effect = effect - effect[0]
        for i in range(effect.shape[0]):
            point = Point()
            point.x = effect[i, 0]
            point.y = effect[i, 1]
            point.z = effect[i, 2]
            marker.points.append(point)

    elif request_name == 'suck' or request_name == 'unscrew':
        marker.ns = request_name+"__effect"
        effect = np.zeros((50, 3))
        effect[:, 2] = np.linspace(0, 0.05, 50)
        for i in range(effect.shape[0]):
            point = Point()
            point.x = effect[i, 0]
            point.y = effect[i, 1]
            point.z = effect[i, 2] + 0.03
            marker.points.append(point)

    rate = rospy.Rate(100)
    for i in range(5):
        # print marker
        effect_publisher.publish(marker)
        rate.sleep()
    return EffectMarkerResponse(True)

def effect_marker_server():
    global effect_publisher
    rospy.init_node('effect_marker_server')
    s = rospy.Service('effect_marker', EffectMarker, handle_effect_marker)
    effect_publisher = rospy.Publisher('asc/lever_up_effect', Marker, queue_size=10)

    print "Effect Marker is ready."
    rospy.spin()


if __name__ == '__main__':
    effect_marker_server()
    rospy.spin()
