#!/usr/bin/env python  
import rospy

import tf_conversions
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
import math

def sendTFStamped(x, y, z, q0, q1, q2, q3, frame, child):
    br_tmp = tf2_ros.TransformBroadcaster()
    t_tmp = geometry_msgs.msg.TransformStamped()
    t_tmp.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
    t_tmp.header.frame_id = frame
    t_tmp.child_frame_id = child
    t_tmp.transform.translation.x =x
    t_tmp.transform.translation.y= y
    t_tmp.transform.translation.z= z
    t_tmp.transform.rotation.x  = q0
    t_tmp.transform.rotation.y  = q1
    t_tmp.transform.rotation.z  = q2
    t_tmp.transform.rotation.w  = q3
    br_tmp.sendTransform(t_tmp)

def odometryCb(msg):
#    br_modom = tf2_ros.TransformBroadcaster()
#    br_obasefoot = tf2_ros.TransformBroadcaster()
#    br_footbase = tf2_ros.TransformBroadcaster()
#    t_modom = geometry_msgs.msg.TransformStamped()
#    t_ofoot = geometry_msgs.msg.TransformStamped()
#    t_footbase = geometry_msgs.msg.TransformStamped()
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    v_x = msg.twist.twist.linear.x
    v_y = msg.twist.twist.linear.y
    w = msg.twist.twist.angular.z
    q0 = msg.pose.pose.orientation.x
    q1 = msg.pose.pose.orientation.y
    q2 = msg.pose.pose.orientation.z
    q3 = msg.pose.pose.orientation.w
#    t_modom.header.stamp = rospy.Time.now()
#    t_modom.header.frame_id = "map"
#    t_modom.child_frame_id = "odom"
#    t_modom.transform.translation.x =x
#    t_modom.transform.translation.y= y
#    t_modom.transform.translation.z= z
#    t_modom.transform.rotation.x  = q0
#    t_modom.transform.rotation.y  =q1
#    t_modom.transform.rotation.z  = q2
#    t_modom.transform.rotation.w  = q3

#    t_ofoot.header.stamp = rospy.Time.now()
#    t_ofoot.header.frame_id = "odom"
#    t_ofoot.child_frame_id = "base_footprint"
#    t_ofoot.transform.translation.x = 0.0
#    t_ofoot.transform.translation.y= 0.0
#    t_ofoot.transform.translation.z= 0.0
#    t_ofoot.transform.rotation.x  = 0.0
#    t_ofoot.transform.rotation.y  = 0.0
#    t_ofoot.transform.rotation.z  = 0.0
#    t_ofoot.transform.rotation.w  = 1.0
    
#    t_footbase.header.stamp = rospy.Time.now()
#    t_footbase.header.frame_id = "base_footprint"
#    t_footbase.child_frame_id = "base_link"
#    t_footbase.transform.translation.x = 0.0
#    t_footbase.transform.translation.y= 0.0
#    t_footbase.transform.translation.z= 0.05
#    t_footbase.transform.rotation.x  = 0.0
#    t_footbase.transform.rotation.y  = 0.0
#    t_footbase.transform.rotation.z  = 0.0
#    t_footbase.transform.rotation.w  = 1.0

#    br_modom.sendTransform(t_modom)
#    br_obasefoot.sendTransform(t_ofoot)
#    br_footbase.sendTransform(t_footbase)
    sendTFStamped(x, y, z, q0, q1, q2, 1, "map", "odom")
    sendTFStamped(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1, "odom", "base_footprint")
    sendTFStamped(0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1, "base_footprint", "base_link")


if __name__ == '__main__':
    rospy.init_node('allTFpublisher')
    #sub = rospy.Subscriber("leobot_base/odom", Odometry, odometryCb)
    rospy.logerr("allTFpublisher disabled")
    #rospy.spin()
