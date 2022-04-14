#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 13:10:24 2020
@author: Martin Sereinig, UIBK
This script can be used at startup to align the world frame of vicon with the world frame of the robot by moving the map
frame, which was initially centered an [0,0,0] to the position where the robot was initialized relative to the world frame f vicon
"""

import sys
import copy
import rospy
import roslib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

from math import pi
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, TransformStamped, Twist, PoseStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix, quaternion_from_matrix, translation_matrix, translation_from_matrix, concatenate_matrices, inverse_matrix 
import tf2_ros


class ViconData(object):
    def __init__(self):
        # Subscribers
        rospy.Subscriber('/vicon/leobot/leobot', TransformStamped, self.vicon_callback)

        self.vicon_x = 0.0
        self.vicon_y = 0.0
        self.vicon_z = 0.0
        self.vicon_roll = 0.
        self.vicon_pitch = 0.
        self.vicon_yaw = 0.
        self.orientation_list = [0.0, 0.0, 0.0, 1.0]

    def vicon_callback(self, data):
        # position
        self.vicon_x = data.transform.translation.x
        self.vicon_y = data.transform.translation.y
        self.vicon_z = data.transform.translation.z
        # euler angles from quaternion
        self.orientation_list = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z,
                            data.transform.rotation.w]
        (self.vicon_roll, self.vicon_pitch, self.vicon_yaw) = euler_from_quaternion(self.orientation_list)

def myshutdownhook():
    print "shutdown time!"


def TFStamped(x, y, z, q, frame, child):
    t_tmp = geometry_msgs.msg.TransformStamped()
    t_tmp.header.stamp = rospy.Time.now()
    t_tmp.header.frame_id = frame
    t_tmp.child_frame_id = child
    t_tmp.transform.translation.x = x
    t_tmp.transform.translation.y = y
    t_tmp.transform.translation.z = z
    t_tmp.transform.rotation.x = q[0]
    t_tmp.transform.rotation.y = q[1]
    t_tmp.transform.rotation.z = q[2]
    t_tmp.transform.rotation.w = q[3]
    return t_tmp

if __name__ == "__main__":
    rospy.init_node('viconAsWorldTF')
    myNode = ViconData()
    br = tf2_ros.TransformBroadcaster()

    loop_rate = rospy.Rate(10)  # Node cycle rat in Hz

    rospy.loginfo('viconASWorldTF waiting for initialization. ')
    rospy.wait_for_message('/vicon/leobot/leobot', geometry_msgs.msg.TransformStamped)
    loop_rate.sleep() # sleep once to obtain zero pose

    q_np = np.array(myNode.orientation_list)
    # switching world and map invertes the transformation!
    tf0 = TFStamped(myNode.vicon_x, myNode.vicon_y, 0, q_np, "world", "map")

    i = 0
    rospy.loginfo('TF viconASWorldTF Started')
    while not rospy.is_shutdown():
        loop_rate.sleep()
        tf0.header.seq = i
        tf0.header.stamp = rospy.Time.now()
        i += 1
        br.sendTransform(tf0)
    rospy.on_shutdown(myshutdownhook)
