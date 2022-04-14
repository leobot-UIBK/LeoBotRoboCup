#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 13:10:24 2020

@author: slyvirtual
"""
import rospy
import franka_modules.panda_grip_modules as gripmodules


# Usage of Grip Module:


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        print('rosnodeinit')
        rospy.init_node('pandaGripper')
        
        
        print('clientfunction')
        gripmodules.pandagripper_homing_client(grippersimulation=True)
        
        
        print('now move grasp with width= 0.02')
        width=0.02  
        force=10.0    
        speed=0.01  
        gripmodules.pandagripper_grasp_client(width,force,speed,epsilon_inner=0.005,epsilon_outer=0.005,grippersimulation=True)
        
        print('now move to 0.03')
        width=0.03
        speed=0.01
        gripmodules.pandagripper_move_client(width, grippersimulation=True)

        
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        