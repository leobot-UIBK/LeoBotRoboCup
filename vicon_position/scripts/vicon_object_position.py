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
import moveit_msgs.msg as moveit_msg
import franka_movement_msgs.srv as franka_srv
import shape_msgs.msg as shape_msg
import geometry_msgs.msg
import numpy as np

from math import pi
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, TransformStamped, Twist, PoseStamped
from moveit_commander.conversions import pose_to_list
import tf.transformations as tft

sys.path.insert(1, "/home/robo/catkin_devel/src/franka_movement/scripts")
import planning_scene_example

class ViconData(object):
    def __init__(self):
        # Publisher

        # Subscribers
        rospy.Subscriber('/vicon/object1/object1', TransformStamped, self.object1_callback)
        self.obj1_x = 0.0
        self.obj1_y = 0.0
        self.obj1_z = 0.0
        self.obj1_roll = 0.
        self.obj1_pitch = 0.
        self.obj1_yaw = 0.
        self.obj1_orientation=[0.,0.,0.,0.]

    def flag_callback(self, data):
        self.control_flag = data.data

    def object1_callback(self, data):
        # position
        self.obj1_x = data.transform.translation.x
        self.obj1_y = data.transform.translation.y
        self.obj1_z = data.transform.translation.z
        # euler angles from quaternion
        self.obj1_orientation = [data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z,
                                 data.transform.rotation.w]
        (self.obj1_roll, self.obj1_pitch, self.obj1_yaw) = tft.euler_from_quaternion(self.obj1_orientation)



    def example_colision_obj(self,id):
        # define message
        primitve = shape_msg.SolidPrimitive()
        primitve.type = primitve.BOX
        primitve.dimensions = [0.05, 0.05, 0.05]

        pose = geometry_msgs.msg.Pose()
        pose.position.x = self.obj1_x
        pose.position.y = self.obj1_y
        #pose.position.z = 0.145
        #pose.position.z = self.obj1_z-(0.025+0.019)
        pose.position.z = self.obj1_z

        #q = tft.quaternion_from_euler(pi / 4.0, 0.0, 0.0)
        print(self.obj1_orientation)
        pose.orientation.x = self.obj1_orientation[0]
        pose.orientation.y = self.obj1_orientation[1]
        pose.orientation.z = self.obj1_orientation[2]
        pose.orientation.w = self.obj1_orientation[3]

        collision_object = moveit_msg.CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = id
        collision_object.primitive_poses = [pose]
        collision_object.primitives = [primitve]

        return collision_object

def spawn_cube(obj):
    service = "/planing_scene_node/collision_object"
    rospy.wait_for_service(service)


    add_collision_obj = franka_srv.CollisionObjectRequest()
    add_collision_obj.collision_object = obj
    add_collision_obj.operation = add_collision_obj.ADD         # ADD

    try:
        call_collision_object = rospy.ServiceProxy(service, franka_srv.CollisionObject)
        response = call_collision_object.call(add_collision_obj)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def myshutdownhook():
    print "shutdown time!"


if __name__ == "__main__":
    rospy.init_node('vicon_object_position', anonymous=True)
    myNode = ViconData()
    name="box"
    ZERO=0.001
    loop_rate = rospy.Rate(10)  # Node cycle rat in Hz
    #while not rospy.is_shutdown():
    #    loop_rate.sleep()

    rospy.sleep(1.0)
    obj=myNode.example_colision_obj(name)
    spawn_cube(obj)


    rospy.on_shutdown(myshutdownhook)
