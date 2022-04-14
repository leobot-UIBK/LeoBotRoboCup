#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon May 17 09:15:52 2021

@author: robo
"""


import rospy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
import tf
import roslib
import sys
import moveit_commander
import moveit_msgs.msg

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

#import franka_modules.panda_grip_modules as gripmodules


def myshutdownhook():
    print "shutdown time!"

#%%
# Just different positions to test
def positions_test(char_inp,moveGroupArm,moveGroupHand):

    if char_inp=='1':
        #configuration drive
        jointDrive=[0.0,-1.4,0.0,-2.8,0.0,2.9,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointDrive)

    if char_inp=='2':
        #Start configuration over desk front
        jointDeskFront=[0.0,-0.2,0.0,-1.8,0.0,1.6,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointDeskFront)

    if char_inp=='3':
        #Start configuration over desk left
        jointDeskLeft=[1.57,0.15,0.0,-1.4,0.0,1.6,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointDeskLeft)

    if char_inp=='4':
        #Start configuration over desk right
        jointDeskRight=[-1.57,0.15,0.0,-1.4,0.0,1.6,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointDeskRight)

    if char_inp=='5':
        #Start configuration over transport plate left
        jointPlateLeft=[2.8,-0.58,0,-2.8,0,2.3,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointPlateLeft)

    if char_inp=='6':
        #Start configuration over transport plate center right from left (ellbow right)
        jointPlateRightCenter=[1.5,-1.3,1.0,-1.9,1.1,1.2,0.22]
        planning_joint_goal(moveGroupArm, setJointGoal=jointPlateRightCenter)


    if char_inp=='7':
        #Start configuration over transport plate right
        jointPlateRight=[-2.8,-0.58,0,-2.8,0,2.3,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointPlateRight)

    if char_inp=='8':
        #Start configuration over transport plate center left from right (ellbow left)
        jointPlateLeftCenter=[-1.6,-1.5,-1.14,-2.19,-1.32,1.3,1.04]
        planning_joint_goal(moveGroupArm, setJointGoal=jointPlateLeftCenter)

    if char_inp=='a':
        #Start configuration over shelf front bottom
        jointShelfBottomFront=[0.0,-0.1,0.0,-2.6,0.0,2.7,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointShelfBottomFront)

    if char_inp=='b':
        #Start configuration over shelf left bottom
        jointShelfBottomLeft=[1.57,-0.1,0.0,-2.6,0.0,2.7,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointShelfBottomLeft)

    if char_inp=='c':
        #Start configuration over shelf right bottom
        jointShelfBottomRight=[-1.57,-0.1,0.0,-2.6,0.0,2.7,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointShelfBottomRight)

    if char_inp=='d':
        #Start configuration over shelf front top
        jointShelfTopFront=[0.0,-0.2,0.0,-1.8,0.0,2.22,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointShelfTopFront)

    if char_inp=='e':
        #Start configuration over shelf left top
        jointShelfTopLeft=[1.57,-0.2,0.0,-1.8,0.0,2.22,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointShelfTopLeft)

    if char_inp=='f':
        #Start configuration over shelf right top
        jointShelfTopRight=[-1.57,-0.2,0.0,-1.8,0.0,2.22,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointShelfTopRight)

    if char_inp=='k':

        object2find = raw_input("your stack object choice (1_F20_20): ") # in python 3 this is only input()
        zDistance=-0.1
        #object2find='1_F20_20_B'
        move2objStackpose(object2find,zDistance,moveGroupArm,scene)
        moveEE(moveGroupArm,[0,0,-1*zDistance,0,0,0])
        raw_input("Press Enter")
        moveEE(moveGroupArm,[0,0,zDistance,0,0,0])




    if char_inp=='u':
        #
        anypose = raw_input("your pose [x,y,z,rotx,roty,rotz]: ") # in python 3 this is only input()
        anypose=eval(anypose)
        anyposePose=['panda_link0']+anypose
        #makeAllObjStack(scene,onoffToggle='off')
        planning_cartesian_goal(moveGroupArm,anyposePose)        # if eef_link='panda_hand'

    if char_inp=='m':
        movementEE=raw_input("movementEE: [x,y,z,rx,ry,rz]")
        moveEE(moveGroupArm,eval(movementEE))

    if char_inp=='s':
        movetask=raw_input("movementEE: 0,1,2,3)")
        moveEE2seach(moveGroupArm,eval(movetask))

    if char_inp=='e':
        cartesianGoal=["world"]+[ 0.5, 0.5, 0.5]+[np.pi/4, 0, 0]
        task='pick'
        pre_gripper_width=0.035
        post_gripper_width=0.025
        distance=0.1
        object_id = ''
        stack='1_F20_20_B'
        pickplaceMove(scene,moveGroupArm,task,cartesianGoal,pre_gripper_width,post_gripper_width,distance,object_id,stack,grippersimulation=True)



#%%
# main function
if __name__=="__main__":
    rospy.init_node('allPositions')
    print('initdone')
    (robot, scene, moveGroupArm, moveGroupHand) = panda_init()
    loop_rate=rospy.Rate(10) #Node cycle rat in Hz
    try:
        loop_rate.sleep()  # callback will be executed


        ## create complete aren
        if makeCompetitionArena(scene):
            print('ARENA READY')
            addAllWallsEnvironment(scene)

        char_inp='g'
        #makeAllObjStack(scene,moveGroupArm,onoffToggle='off')
        while char_inp!='x' or not rospy.is_shutdown():
            print('choose your config')
            print("Drive (1),DeskFront (2),DeskLeft (3),Deskright (4), PlateLeft (5),")
            print("PlateRightCenter (6), PlateRight (7), PlateLeftCenter (8),")
            print("ShelfBottomFront (a), ShelfBottomLeft (b), ShelfBottomRight (c)")
            print("ShelfTopFront (d), ShelfTopLeft (e), ShelfTopRight (f)")
            print("k for stack positions")
            print("u for any POSE")
            print("m for inspection")
            print("s for seach in ee frame")
            print("e for pick")
            print("Stop press (x)")

            char_inp = raw_input("your choice: ") # in python 3 this is only input()


            makeAllObjStack(scene,moveGroupArm,onoffToggle='on')  # turn on all obstackle stacks
            #makeAllObjStack(scene,onoffToggle='off')  # turn of all obstackle stacks
            positions_test(char_inp,moveGroupArm,moveGroupHand)


            if char_inp in ['x','q', 'Q', 'n', 'N']:
                print('done')
                break

            if rospy.is_shutdown():
                print('shutdown')
                break


    except rospy.ROSInterruptException:
        print("Shutting down")
        rospy.on_shutdown(myshutdownhook)
