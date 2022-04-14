#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 13:10:24 2020

@author: slyvirtual
"""

import rospy
import numpy as np
import roslib
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
     nextexecution)

#import franka_modules.panda_move_modules as fm
#fm.panda_init()


def myshutdownhook():
    print "shutdown time!"

if __name__=="__main__":
    rospy.init_node('pandamovetest',anonymous=True)
    
    (robot, scene, moveGroupArm, moveGroupHand)=panda_init()
    loop_rate=rospy.Rate(10) #Node cycle rat in Hz
    
    sceneObjects=moveit_commander.PlanningSceneInterface.get_known_object_names(scene)
    if 'base_box' in sceneObjects:
        scene.remove_attached_object('panda_link0',name='base_box')
        wait_for_state_update('base_box',scene)
        scene.remove_world_object('base_box')
        wait_for_state_update('base_box',scene)
    
    rothand=-np.pi/4
    while not rospy.is_shutdown():

        # add plane in world
#        addPlaneEnvironment('world_plane',adscene=scene,pose=["world", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],normal=[0.0,0.0,1.0])
#        wait_for_state_update('world_plane',scene)
#        moveGroupArm.attach_object('world_plane',link_name='panda_link0')    # 'panda_link8'
#        wait_for_state_update('world_plane',scene)
        planeDimensions=[4,4,0.001]
        addBoxEnvironment('labPlane',pose=['world',0,0,-1.0,0,0,0],dimension=planeDimensions,adscene=scene) 
        #moveGroupArm.attach_object('labTable',link_name='panda_link0')    # 'panda_link8'
        wait_for_state_update('labPlane',scene)           
        
        
        
        # add laboratory table
        tableDimensions=[1.94,1.,0.01]
        addBoxEnvironment('labTable',pose=['world',0,0,-0.011,0,0,0],dimension=tableDimensions,adscene=scene) 
        #moveGroupArm.attach_object('labTable',link_name='panda_link0')    # 'panda_link8'
        wait_for_state_update('labTable',scene)   
        
        
          # add laboratory wall left
        wallDimensions=[1.25,0.0001,1.75]
        addBoxEnvironment('wallLeft',pose=['world',0,-0.75,wallDimensions[2]/2,0,0,0],dimension=wallDimensions,adscene=scene)
        wait_for_state_update('wallLeft',scene)        
        
        
        # add laboratory wall right
        addBoxEnvironment('wallRight',pose=['world',0,0.6,wallDimensions[2]/2,0,0,0],dimension=wallDimensions,adscene=scene)
        wait_for_state_update('wallRight',scene)      



        # add collecting boxes
        collectBoxDim=[0.2,0.125,0.125]
        collectBoxPose=[0.250,-0.400,0,0,0,np.pi/2]
        addBoxEnvironment('collectbox1', 
                          pose=['world',
                                collectBoxPose[0],
                                collectBoxPose[1],
                                collectBoxPose[2],
                                collectBoxPose[3],
                                collectBoxPose[4],
                                collectBoxPose[5]],dimension=collectBoxDim,adscene=scene) 
        wait_for_state_update('collectbox1',scene)   
        
        addBoxEnvironment('collectbox2', 
                          pose=['world',
                                collectBoxPose[0]+0.200,
                                collectBoxPose[1],
                                collectBoxPose[2],
                                collectBoxPose[3],
                                
                                collectBoxPose[4],
                                collectBoxPose[5]],dimension=collectBoxDim,adscene=scene) 
        wait_for_state_update('collectbox2',scene)           

        addBoxEnvironment('collectbox3', 
                          pose=['world',
                                collectBoxPose[0]+0.400,
                                collectBoxPose[1],
                                collectBoxPose[2],
                                collectBoxPose[3],
                                collectBoxPose[4],
                                collectBoxPose[5]],dimension=collectBoxDim,adscene=scene) 
        wait_for_state_update('collectbox3',scene)         

        # add manipulatio object       
        box1Dim=[0.05,0.05,0.05]
        addBoxEnvironment('box1',pose=['world',0.5,0,box1Dim[2]/2+0.001,0,0,0],dimension=box1Dim,adscene=scene) 
        wait_for_state_update('box1',scene)
        
        
        
        print "============ Start the motion to joint goal"
        if nextexecution():
            break
      
        planning_joint_goal(moveGroupArm, setJointGoal=[0, -np.pi/4, 0, -np.pi/2, 0, np.pi/3, 0])
        print "============ Press `Enter` to start the motion to cartesian goal"
        if nextexecution():
            break  
        #planning_cartesian_goal(moveGroupArm,['world',0.25,0,0.7,-np.pi,0,0+rothand])        # if eef_link='panda_link8'     
        planning_cartesian_goal(moveGroupArm,['world',0.25,0,0.7,-np.pi,0,0])        # if eef_link='panda_hand'           
        
      
        #just to open the gripper bevor grasping
        planning_joint_goal(moveGroupHand, setJointGoal=[0.038,0.038]) # to open gripper
        print "============ Press `Enter` to start the motion with a cartesian path"
        if nextexecution():
            break
   
        
        
        P=[]
        P.append([0.25,0,0,0,0,0])
        P.append([0,0,-0.675,0,0,0])
        (plan,fraction)=planning_cartesian_path(moveGroupArm,P)
        moveGroupArm.execute(plan, wait=True) #to execute the plan 
        moveGroupArm.attach_object('box1',link_name='panda_hand')    # 'panda_link8'
        #just to close the gripper while grasping
        planning_joint_goal(moveGroupHand, setJointGoal=[0.026,0.026]) # to open gripper
       
        P=[]
        P.append([0,0,0.55,0,0,0])
        P.append([0,0.35,0,0,0,0])
        P.append([-0.05,0,0,0,0,0])
        P.append([-0.35,0,0,0,0,0])
        P.append([0,0,0,0,0,np.pi/2])
        P.append([0,0,-0.349,0,0,0])
        (plan,fraction)=planning_cartesian_path(moveGroupArm,P)
        moveGroupArm.execute(plan, wait=True) #to execute the plan 
        
        #just to open the gripper after grasping
        planning_joint_goal(moveGroupHand, setJointGoal=[0.038,0.038]) # to open gripper        
        scene.remove_attached_object('panda_link8',name='box1')
        wait_for_state_update('box1',scene)
       
        P=[]
        P.append([0,0,0.3,0,0,0])
        P.append([0,0,0,0,0,-np.pi/2])
        P.append([0,-0.25,0,0,0,0])
        (plan,fraction)=planning_cartesian_path(moveGroupArm,P)
        moveGroupArm.execute(plan, wait=True) #to execute the plan 

        
        #remove piced box from arm and from world
        scene.remove_world_object('box1')
        wait_for_state_update('box1',scene)
        
        
        
        

    loop_rate.sleep() # callback will be executed     
    rospy.on_shutdown(myshutdownhook)

