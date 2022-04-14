#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------
#   Measurement Master
#   DESCRIPTION: for running trajectories specified in a measurement file, 
#   created in the format as in CreateMasterInput.py and the example file from 
#   the example file contained in the package. 
#   AUTHOR: Peter Manzl
#   LICENCE: use under the BSD3 permitted. Only for the use of qualified staff 
#   NO WARRANTY or guarantee of any kind is given. 
#   DATE: V1 created on the 31.03.2021 for the LeoBot project. 
# ------------------------------------------------------------------------


import rospy
import std_msgs.msg
from std_msgs.msg import Bool
import numpy as np
import measurement_msgs.msg
import measurement_msgs.srv
import sys # for arguments 
import time 

# fills the data into the message of type measurement_flag, consisting of data and value. 
def createMeasMsg(msg, i, val): 
    msg.header.frame_id = '' # no frame associated
    msg.header.seq = i
#    t = rospy.get_time()
    msg.header.stamp = rospy.Time.now() # obtain ros internal time 
    msg.flag = val

# send the data to the Trajectory service
def TrajService_client(data):
    rospy.wait_for_service('timedVel')
    try:
        TrajService = rospy.ServiceProxy('timedVel', measurement_msgs.srv.timedVel)
        # create handle for calling the service
        rospy.loginfo('Execution of Trajectory started.')
        resp = TrajService(data) # use handle
        return resp # resp is a timedVel response object, so resp.successcontains the float. 
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: %s"%e) # exception is thrown if the call fails
        return -1 

# main function   
def MeasurementMaster(datafile):
    
    rospy.init_node('MeasurementMaster', anonymous=False) # init node
    pubControl = rospy.Publisher('/vicon/leobot/leobot_control_flag', Bool, queue_size=2)
    pubMeasFlag = rospy.Publisher('/meassurements/flag', measurement_msgs.msg.measurement_flag, queue_size=2)
    try: 
        rospy.loginfo('Datafile ' + datafile)
        data = np.loadtxt(datafile, comments = '#', delimiter=',')
    except IOError: 
        rospy.loginfo('The specified Datafile ' + datafile +' was not found. ')
        return 0
    n_size = data.shape[0]
    i_meas = 0
    ros_t0 = rospy.get_time()
    msgMeasurementFlag = measurement_msgs.msg.measurement_flag()
    str_Traj = 'i ' + '\t' + 'sx ' + '\t' + 'sy ' + '\t' + 'phi ' + '\t' + \
                'vx ' + '\t' + 'vy ' + '\t' + 'w  ' + '\t' + \
                'ax  ' + '\t' + 'ay ' + '\t' + 'dw ' + '\t' + 'newMeas' + '\n'
    skip = False
    while not rospy.is_shutdown() and (i_meas < n_size):
        TrajService = rospy.ServiceProxy('/leobot_base/TrajectorySrv', measurement_msgs.srv.timedVel)
        movement = data[i_meas,:-1] # last element indicates if trajectory is last element of movement and robot should reset to zero position.
        flagNewMeas  = int(data[i_meas,-1])
        if flagNewMeas  == 1: 
            time.sleep(0.2)
            rospy.loginfo('Driveback to zero started.')
            pubControl.publish(1) # restart control to move back to zero position
            createMeasMsg(msgMeasurementFlag, i_meas, 3)
            pubMeasFlag.publish(msgMeasurementFlag) # 3 -> end driving back to  zero and continue
            skip = False
            flagConfirm = True
        # confirm new measurement
        str_mov = ''
        readMeas = True
        j = 0
        while (readMeas and flagConfirm):  #  read data of measurement
            if(skip): 
                break
            
            for i in range(data.shape[1]):      
                if i == 0: 
                    str_mov += str(int(data[i_meas+j, i])) + '\t'
                elif i == 10: # flagZero 
                        if data[i_meas+j,i] == 1:
                            str_mov += 'true'+ '\n'
                        else: 
                            str_mov += 'false'+ '\n'
                            #false
                else: 
                    str_mov += str(round(data[i_meas, i],2)) + '\t'
            try: 
                if data[i_meas+j+1, 10] == 1: 
                    readMeas = False
            except: 
                    readMeas = False
            j+=1
        if flagConfirm: 
            rospy.loginfo(('Skipped ' * skip) + ('Confirm ' * bool(not(skip))) +  \
                       'Trajectory '+ str(i_meas) + ':\n ' * bool(not(skip)) \
                      + (str_Traj + str_mov)*bool(not(skip)))
        elif not(flagConfirm):
            rospy.loginfo('Trajecotry ' + str(i_meas) + ' is in execution.')
        if not(skip): 
            while(1 and flagConfirm): 
                char_inp = raw_input("press y (yes) /n (no) / s (skip): ") # in python 3 this is only input()
                if char_inp in ['y', 'Y']:
                    skip = False
                    flagConfirm = False
                    break
                elif char_inp in ['q', 'Q', 'd', 'D', 'n', 'N']: 
                    rospy.loginfo('Trajectory terminated because of user input.')
                    return
                elif char_inp in ['s', 'S']: 
                    rospy.loginfo('Skipped Trajectory ' + str(i_meas))
                    skip = True  
                    break
                else: 
                    rospy.loginfo('Invalid input character. ')

            
        
            # rospy.loginfo(str(i_meas))
            if not(skip): 
                time.sleep(0.2)
                createMeasMsg(msgMeasurementFlag, i_meas, 1)
                pubMeasFlag.publish(msgMeasurementFlag) # 1 -> start measurement
                pubControl.publish(0)
                retVal = TrajService(data[i_meas,:-1])
                if pubControl == -1: 
                    rospy.loginfo('Trajectory Service failed. The parameters may be not valid. ')
                rospy.loginfo('Trajectory ' + str(i_meas) + ' finished after ' + \
                              str(retVal.time) + 'seconds.')
                createMeasMsg(msgMeasurementFlag, i_meas, 2)
                pubMeasFlag.publish(msgMeasurementFlag) # 2-> end of trajectory 

        i_meas +=1
      #   rate.sleep()
    rospy.loginfo('Measurement finished after ' + str(i_meas) + ' movements.')
    return 1
# target rausschreiben auf 
      
if __name__ == '__main__':
    try:
        if isinstance(sys.argv[1], str) and isinstance(sys.argv[2], str): 
            datafile = sys.argv[1] + '/' + sys.argv[2]
        retVal = MeasurementMaster(datafile)
        rospy.signal_shutdown('MeasurementMaster finished with code ' + str(retVal) + '. ')
    except rospy.ROSInterruptException:
        pass
