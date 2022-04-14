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
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Twist, PoseStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion #quaternion calculation from euler angels



#QR Code Position 
class QRData(object):
	def __init__(self):
		#Subscribers
		rospy.Subscriber('/visp_auto_tracker_rechts/object_position', PoseStamped, self.callback_pose)
		rospy.Subscriber('/visp_auto_tracker_rechts/code_message', String, self.callback_info)		 #not sure yet

	def callback_pose(self,data):
		# position
		self.variable_x = data.pose.position.x
		self.variable_y = data.pose.position.y
		self.variable_z = data.pose.position.z
		# euler angles from quaternion 
		self.orientation_list = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (self.orientation_list)

	def callback_info(self,data):
		self.massage = data.data



def myshutdownhook():
    print("shutdown time!")


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


if __name__=="__main__":
    rospy.init_node('qr_position', anonymous=True)
    my_node=QRData()
    tf2broadcaster = tf2_ros.TransformBroadcaster()
    i = 0
    while not rospy.is_shutdown():   
    	
        if my_node.massage=="START":

            loop_rate = rospy.Rate(10)  # Node cycle rat in Hz
            rospy.loginfo('viconASWorldTF waiting for initialization. ')
            rospy.wait_for_message('/vicon/leobot/leobot', geometry_msgs.msg.TransformStamped)
            loop_rate.sleep() # sleep once to obtain zero pose
        
            q_np = np.array(myNode.orientation_list)
            # switching world and map invertes the transformation!
            tf0 = TFStamped(myNode.variable_x, myNode.variable_y, myNode.variable_z, q_np, "world", "map")
            rospy.loginfo('TF viconASWorldTF Started')
            tf0.header.seq = i
            tf0.header.stamp = rospy.Time.now()
            i += 1
            tf2broadcaster.sendTransform(tf0)
    
    rospy.on_shutdown(myshutdownhook)




