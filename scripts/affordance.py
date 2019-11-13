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
import copy
from geometry_msgs.msg import TransformStamped,Pose
import tf2_ros
import numpy as np
import rospack

## Test This
def comparePosition(leverup_pose,old_pcb_pose, new_pcb_pose):
    ### Lever Up Pose

    buffer_core = tf2_ros.BufferCore(rospy.Duration(1.0))
    ts1 = TransformStamped()
    ts1.header.stamp = rospy.Time(0)
    ts1.header.frame_id = 'map'
    ts1.child_frame_id = 'frame1'
    ts1.transform.translation=old_pcb_pose.position

    ts1.transform.rotation=leverup_pose.orientation

    buffer_core.set_transform(ts1, "default_authority")

    traj = np.loadtxt(rospack.get_path('imagine_asc') + "/10_10.txt") / 4.
    ### Trajectory
    ts2 = TransformStamped()
    ts2.header.stamp = rospy.Time(0)
    ts2.header.frame_id = 'frame1'
    ts2.child_frame_id = 'frame2'

    traj[-1,:2]-traj[0,:2]
    ts2.transform.translation.y = 2*(traj[-1,0]-traj[0,0])
    ts2.transform.translation.x = 2*(traj[-1,1]-traj[0,1])
    ts2.transform.rotation.w = 1.0
    buffer_core.set_transform(ts2, "default_authority")

    transformation = buffer_core.lookup_transform_core('map', 'frame2', rospy.Time(0))
    return np.linalg.norm([new_pcb_pose.position.x-transformation.transform.translation.x,new_pcb_pose.position.y-transformation.transform.translation.y])

class affordanceWrapper:
    comp_img = []
    affordance_vis_image = []

    def __init__(self):
        self.affordance_list = []
        self.affordance_funcs = {}
        self.available_parts = []
        self.required_parts_for_affordance = {}
        self.s = rospy.Service('asc/request_affordances', AffordanceArray, self.requestAffordances)        ##TODO affordances --> production, affordances2 --> debug
        self.affordances_image_rviz = rospy.Publisher('asc/debug/affordances_image/compressed', CompressedImage, queue_size=10)
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.curr_poses=None
        self.old_poses=None
        self.last_action=None
        self.last_action_pose=None

    def updateDataRequest(self, request):
        self.old_poses = copy.deepcopy(self.curr_poses)
        self.curr_data = request
        self.available_parts = list()
        tmp_img = self.bridge.imgmsg_to_cv2(self.curr_data.assos_img, self.curr_data.assos_img.encoding)
        affordanceWrapper.affordance_vis_image = cv2.resize(tmp_img, (256, 256))

        self.curr_poses = dict()
        for part in self.curr_data.part_array:
            partname = part.part_id[:-1]
            self.available_parts.append(partname)
            self.curr_poses[partname] = copy.deepcopy(part.pose) ## ToDo: Part_id

    def addAffordanceModel(self, affordance):
        aff = affordance.affordance
        if aff not in self.affordance_list:
            self.affordance_list.append(aff)
        self.affordance_funcs[aff] = affordance.find_affordance
        self.required_parts_for_affordance[aff] = affordance.required_parts_for_affordance

    def requestAffordances(self, request):
        self.updateDataRequest(request)

        #### Mismatch Check
        if self.last_action=='lever':
            action_success=True
            if 'pcb' in self.curr_poses.keys():
                position_diff = comparePosition(self.last_action_pose,self.old_poses['pcb'],self.curr_poses['pcb'])
                if position_diff>0.025:
                   action_success=False
            if action_success:
                print('========= PCB leverup action was SUCCESSFUL according to ASC effect prediction ===========')
            else: 
                print('======== PCB leverup action was FAILURE according to ASC effect prediction ===========')
        ####

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
