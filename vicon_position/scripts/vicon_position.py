#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 13:10:24 2020
@author: slyvirtual
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
from tf.transformations import quaternion_from_euler, euler_from_quaternion  # quaternion calculation from euler angel


class ViconData(object):
    def __init__(self):
        # Publisher
        self.pup = rospy.Publisher("/leobot_base/cmd_vel", Twist, queue_size=10)
        self.twistObject = Twist()
        # Subscribers
        rospy.Subscriber('/vicon/leobot/leobot', TransformStamped, self.vicon_callback)
        rospy.Subscriber('/leobot_base/target_position', Pose, self.target_callback)
        rospy.Subscriber('/vicon/leobot/leobot_control_flag', Bool, self.flag_callback)

        self.target_x = 0.
        self.target_y = 0.
        self.target_z = 0.
        self.target_roll = 0.
        self.target_pitch = 0.
        self.target_yaw = 0.

        self.vicon_x = 0.0
        self.vicon_y = 0.0
        self.vicon_z = 0.0
        self.vicon_roll = 0.
        self.vicon_pitch = 0.
        self.vicon_yaw = 0.
        self.control_flag = False

    def flag_callback(self, data):
        self.control_flag = data.data

    def vicon_callback(self, data):
        # position
        self.vicon_x = data.transform.translation.x
        self.vicon_y = data.transform.translation.y
        self.vicon_z = data.transform.translation.z
        # euler angles from quaternion
        orientation_list = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z,
                            data.transform.rotation.w]
        (self.vicon_roll, self.vicon_pitch, self.vicon_yaw) = euler_from_quaternion(orientation_list)

    def target_callback(self, data):
        # position
        self.target_x = data.position.x
        self.target_y = data.position.y
        self.target_z = data.position.z
        # euler angles from quaternion
        orientation_list = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (self.target_roll, self.target_pitch, self.target_yaw) = euler_from_quaternion(orientation_list)

    def WorldRotation(self, theta):
        c, s = np.cos(theta), np.sin(theta)
        rotWorld = np.array(((c, -s), (s, c)))
        return rotWorld


def myshutdownhook():
    print "shutdown time!"


if __name__ == "__main__":
    rospy.init_node('vicon_position', anonymous=True)
    myNode = ViconData()
    ZERO=0.001
    loop_rate = rospy.Rate(10)  # Node cycle rat in Hz
    while not rospy.is_shutdown():
        loop_rate.sleep()

        controllerErrorx = myNode.target_x - myNode.vicon_x
        controllerErrory = myNode.target_y - myNode.vicon_y
        controllerErrorz = myNode.target_yaw - myNode.vicon_yaw

        controllerGainx = 0.8
        controllerGainy = 0.8
        controllerGainz = 0.8
        controllerSpeedx = controllerGainy * controllerErrorx
        controllerSpeedy = controllerGainy * controllerErrory
        controllerSpeedz = controllerGainz * controllerErrorz

        Rotmat = myNode.WorldRotation(myNode.vicon_yaw)
        veloRobotxy = np.matmul(Rotmat.transpose(), np.array(((controllerSpeedx), (controllerSpeedy))))
        veloRobotz = controllerSpeedz

        #rospy.loginfo('Heard Vicon x= %f', myNode.vicon_x)
        #rospy.loginfo('Heard Vicon y= %f', myNode.vicon_y)
        #rospy.loginfo('Heard Vicon rotx= %f', myNode.vicon_yaw)

        #print(myNode.control_flag)
        if myNode.control_flag and (abs(float(veloRobotxy[0]))>ZERO or  abs(float(veloRobotxy[1]))>ZERO or abs(float(veloRobotz))>ZERO):
            print('controller on')
            rospy.loginfo('vx = %f', float(veloRobotxy[0]))
            rospy.loginfo('vy = %f', float(veloRobotxy[1]))
            rospy.loginfo('wz = %f', float(veloRobotz))
            myNode.twistObject.linear.x = float(veloRobotxy[0])
            myNode.twistObject.linear.y = float(veloRobotxy[1])
            myNode.twistObject.angular.z = float(veloRobotz)
            myNode.pup.publish(myNode.twistObject)
        else:
            print('controller off')
            myNode.twistObject.linear.x = 0.0
            myNode.twistObject.linear.y = 0.0
            myNode.twistObject.angular.z = 0.0


    rospy.on_shutdown(myshutdownhook)
