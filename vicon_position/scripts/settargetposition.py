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
import numpy

from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Twist, PoseStamped
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler, euler_from_quaternion #quaternion calculation from euler angels



class SetTargetPos(object):
    def __init__(self):
		#Publisher
        self.pup=rospy.Publisher("/leobot_base/target_position",Pose, queue_size=10)
        self.PoseObject=Pose()
        self.PoseObject.position.x=0.0
        self.PoseObject.position.y=-1.0
        self.PoseObject.position.z=0.0
        # Make sure the quaternion is valid and normalized
        self.rotx=0
        self.roty=0
        self.rotz=0
        self.quaternion = quaternion_from_euler(self.rotx, self.roty, self.rotz)
        self.PoseObject.orientation.x = self.quaternion[0]
        self.PoseObject.orientation.y = self.quaternion[1]
        self.PoseObject.orientation.z = self.quaternion[2]
        self.PoseObject.orientation.w = self.quaternion[3]

        #Subscribers

def myshutdownhook():
    print "shutdown time!"

if __name__=="__main__":
    rospy.init_node('settargetposition', anonymous=True)
    my_node=SetTargetPos()


    loop_rate=rospy.Rate(10) #Node cycle rat in Hz
    rospy.loginfo("Data shown:\n")
    while not rospy.is_shutdown():

        if isinstance(sys.argv[1],str):
            targetdata=sys.argv[1]
            targetdata = [[float(x.strip(' ')) for x in ss.lstrip(' [,').split(',')] for ss in targetdata.rstrip(']').split(']')]
            my_node.PoseObject.position.x=targetdata[0][0]
            my_node.PoseObject.position.y=targetdata[0][1]
            my_node.rotz=targetdata[0][2]
            my_node.quaternion=quaternion_from_euler(my_node.rotx,my_node.roty,my_node.rotz)
            my_node.PoseObject.orientation.x = my_node.quaternion[0]
            my_node.PoseObject.orientation.y = my_node.quaternion[1]
            my_node.PoseObject.orientation.z = my_node.quaternion[2]
            my_node.PoseObject.orientation.w = my_node.quaternion[3]

        loop_rate.sleep()
        rospy.loginfo('Target position x= %f', my_node.PoseObject.position.x)
        rospy.loginfo('Target position y= %f', my_node.PoseObject.position.y)
        rospy.loginfo('Target position rotz= %f', my_node.rotz)
        my_node.pup.publish(my_node.PoseObject)

    rospy.on_shutdown(myshutdownhook)
