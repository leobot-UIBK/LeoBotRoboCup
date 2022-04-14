#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 20 16:40:22 2021

@author: Martin Sereinig
"""

import franka_movement_msgs.srv as franka_srv
import moveit_msgs.msg as moveit_msg
import geometry_msgs.msg
import shape_msgs.msg as shape_msg
import rospy
import numpy as np
import tf
import roslib
import sys
import moveit_commander

from franka_modules.panda_move_modules import (
     panda_init,
     addBoxEnvironment,
     addPlaneEnvironment,
     addCylinderEnvironment,
     planning_joint_goal,
     planning_cartesian_goal,
     planning_cartesian_path,
     wait_for_state_update,
     nextexecution,
     moveEE,
     makeAllObjStack,
     makeObjStack,
     makeCompetitionArena,
     makeArenaWorkStation,
     pickplaceMove,
     move2objStackpose,
     moveEE2seach,
     addAllWallsEnvironment,
     addWallEnvironment)

grippersimulation = False

moveGroupArm = None


def pickplacecallback(req):
    global grippersimulation
    global moveGroupArm
    try:
        if moveGroupArm is None:
            moveGroupArm = moveit_commander.MoveGroupCommander("panda_arm")
            moveGroupArm.set_end_effector_link('panda_gripper_center')

        scene = moveit_commander.PlanningSceneInterface()
        object_id=req.object_id

        distance=req.distance
        pre_gripper_width=req.pre_gripper_width
        post_gripper_width=req.post_gripper_width
        stackname=req.stack_position
        print("stackposition ", stackname)
        stack=stackname.replace('objStack','')


        if req.pick: #true if pick
            task='pick'

        else:
            task='place'
            
        #pose_Goal = geometry_msgs.msg.PoseStamped()
        pose_Goal = req.pose_stamped
        
        Quaternionlist=[pose_Goal.pose.orientation.x, pose_Goal.pose.orientation.y, pose_Goal.pose.orientation.z, pose_Goal.pose.orientation.w]
        eulerNow=list(tf.transformations.euler_from_quaternion(Quaternionlist))
        
        cartesianGoal=[pose_Goal.header.frame_id]+[pose_Goal.pose.position.x, pose_Goal.pose.position.y, pose_Goal.pose.position.z]+eulerNow

        result = pickplaceMove(scene,moveGroupArm, task, cartesianGoal, pre_gripper_width, post_gripper_width, distance,object_id, stack ,grippersimulation)
        if result == franka_srv.PickAndPlaceResponse.SUCCESS:
            print("Pick and Place sucsess")
        else:
            print("Pick and Place error, see result for error code")
        return result
    
    
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return franka_srv.PickAndPlaceResponse(1)

    







if __name__ == "__main__":
    if isinstance(sys.argv[1],str):
        targetdata=sys.argv[1]
        if targetdata=='true':
            grippersimulation=True
            print('assimulation= True')
        else:
            print("assimulation = False")


    rospy.init_node('pick_place')

    s = rospy.Service('~pick_place_service', franka_srv.PickAndPlace, pickplacecallback)
    print("Ready to Pick or Place.")
    rospy.spin()
