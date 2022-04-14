#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 27 13:10:24 2020

@author: slyandsmart
"""

import rospy
import numpy as np
import roslib
import sys
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

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import String
    
def myshutdownhook():
    print "shutdown time!"

def ImageService_client():
    rospy.wait_for_service('/get_image_frame')
    try:
        ImageService = rospy.ServiceProxy('/get_image_frame', Trigger)
        # create handle for calling the service
        rospy.loginfo('Get image started.')
        resp = ImageService() # use handle
        return resp # resp is a flag
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e) # exception is thrown if the call fails
        return -1
    
    

if __name__=="__main__":
    rospy.init_node('frankaTrainingMove')
    (robot, scene, moveGroupArm, moveGroupHand) = panda_init()
    loop_rate=rospy.Rate(10) #Node cycle rat in Hz
    try:
        loop_rate.sleep()  # callback will be executed
        scene.remove_attached_object('panda_link0', name='base_box')
        scene.remove_world_object('base_box')
        wait_for_state_update('base_box', scene)
        
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
        
        # add box as place holder for obstacles right
        obstaclesDimensions=[0.3,0.8,0.1]
        addBoxEnvironment('obstacles',pose=['world',0.5,0,obstaclesDimensions[2]/2,0,0,0],dimension=obstaclesDimensions,adscene=scene)
        wait_for_state_update('obstacles',scene)    
        
        
        
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
        
      
        #Start configuration over desk
        jointDesk=[0.0,-0.2,0.0,-1.8,0.0,1.6,0.785]
        planning_joint_goal(moveGroupArm, setJointGoal=jointDesk)
        angleTild=np.pi/30
        #resp=ImageService_client()
        imag=1
        roundcount=1        
        distanceZ=15    #movement of robot in z direction, 10 means robot goes from initial position down 10cm during the whole process 
        while not rospy.is_shutdown() and roundcount<=distanceZ: 


            PM=([   [0.00, 0.00, -0.01, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, angleTild, 0.00],
                    [0.00, 0.00, 0.00, angleTild, 0.00, 0.00],            
                    [0.08, 0.00, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, -angleTild*2, 0.00],
                    [0.00, 0.00, 0.00, -angleTild*2, 0.00, 0.00],    
                    [0.00, 0.08, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, angleTild, 0.00],
                    [0.00, 0.00, 0.00, angleTild, 0.00, 0.00],    
                    [-0.16, 0.00, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, -angleTild, 0.00],
                    [0.00, 0.00, 0.00, -angleTild, 0.00, 0.00],    
                    [0.00, -0.16, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, angleTild*2, 0.00],
                    [0.00, 0.00, 0.00, angleTild*2, 0.00, 0.00],    
                    [0.08, 0.08, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, -angleTild, 0.00],
                    [0.00, 0.00, 0.00, -angleTild, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, angleTild, 0.00],
                    [0.00, 0.00, 0.00, angleTild, 0.00, 0.00],            
                    [0.08, 0.00, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, -angleTild*2, 0.00],
                    [0.00, 0.00, 0.00, -angleTild*2, 0.00, 0.00],    
                    [0.00, 0.08, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, angleTild, 0.00],
                    [0.00, 0.00, 0.00, angleTild, 0.00, 0.00],    
                    [-0.16, 0.00, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, -angleTild, 0.00],
                    [0.00, 0.00, 0.00, -angleTild, 0.00, 0.00],    
                    [0.00, -0.16, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, angleTild*2, 0.00],
                    [0.00, 0.00, 0.00, angleTild*2, 0.00, 0.00],    
                    [0.08, 0.08, 0.00, 0.00, 0.00, 0.00],
                    [0.00, 0.00, 0.00, 0.00, 0.00, np.pi/6],
                    [0.00, 0.00, 0.00, 0.00, -angleTild, 0.00],
                    [0.00, 0.00, 0.00, -angleTild, 0.00, 0.00] 
                    ])   

            #cartesian movement with random variations
            if np.mod(roundcount,2)==1: 
                sigZang=-1
            else:
                sigZang=1
                
            print("now movement")
            print("round",roundcount)
            for i in range(len(PM)):
                    P=[]
                    P.append([np.float64(PM[i][0]+((np.random.rand()-0.5)*2/100)),
                        np.float64(PM[i][1]+((np.random.rand()-0.5)*2/100)),
                        np.float64(PM[i][2]+((np.random.rand()-0.5)*2/1000)),
                        np.float64(PM[i][3]+((np.random.rand()-0.5)*2/100)),
                        np.float64(PM[i][4]+((np.random.rand()-0.5)*2/100)),
                        sigZang*np.float64(PM[i][5]+((np.random.rand()-0.5)*2/100))]) 
                    (plan,fraction)=planning_cartesian_path(moveGroupArm,P)
                    moveGroupArm.execute(plan, wait=True) #to execute the plan
                    imag+=1
                    print('image: ',imag)
                    resp=ImageService_client()
                    if rospy.is_shutdown():
                        print('shutdown by user interrupt')
                        break
            roundcount+=1






    except KeyboardInterrupt:
        print("Shutting down")
    rospy.on_shutdown(myshutdownhook)

        


