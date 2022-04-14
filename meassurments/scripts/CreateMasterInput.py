#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------
#   Measurement Master
#   DESCRIPTION: create datafiles for the MeasurementMaster. 
#   AUTHOR: Peter Manzl
#   LICENCE: use under the BSD3 permitted. Only for the use of qualified staff 
#   NO WARRANTY or guarantee of any kind is given. 
#   DATE: V1 created on the 31.03.2021 for the LeoBot project. 
# ------------------------------------------------------------------------

import numpy as np
import rospy
import std_msgs.msg
#from std_msgs.msg import Bool
#import measurement_msgs.msg
#import measurement_msgs.srv
import sys # for arguments 
# import keyboard as keyboard # for input query



def CreateMasterInput(): 
    rospy.loginfo('This script is for creating Measurment data for the MeasurmentMaster. \n' + \
                  'To generate different paths edit the .py file. ')
    strPath = '../data/'

    n_wdh = 3 # number of runs per meassurement
    s = [0.3, 0.3, 0.3, 0.3] # length of sides of the rectangle

    #v = np.array([0.2, 0.4, 0.8])
    #a = np.array([0.4, 0.8, 0.8])

    v = np.array([0.2, 0.4])
    a = np.array([0.4, 0.8])

    
    phi = np.array([2*np.pi])
    w = np.array([0.5, 1])    
    dw = np.array([1, 2])
    strFilenameRect = 'Rectangle_' + str(n_wdh) + 'Runs_' + str(v.shape[0]) + 'Velocities.txt'
    strFilenameRot = 'Rotation_' + str(n_wdh) + 'Runs_' + str(v.shape[0]) + 'Velocities.txt'
    str_header = '# This is a inputfile for the MeasurmentMaster. \n' + \
                '# Each line resembles one trajectory. The format is: \n' + \
                '# i \tsx \tsy \tphi \tvx  \tvy \tw \tax  \tay  \tdw  \tnewMeas\n' + \
                '# --------------------------------------------------------------------------------------\n'
    file_Rect  = open(strPath + strFilenameRect, "w+") 
    file_Rect.write(str_header) # header
    file_Rot  = open(strPath + strFilenameRot, "w+") 
    file_Rot.write(str_header)
    l = 0
    for i in range(v.shape[0]): 
        for j in range(n_wdh):
            # one rectangle is front, left, back, right
            file_Rect.write( str(l) + ', \t' + str(s[0]) + ', \t0, \t0, \t'  + str(v[i]) + ', \t0,\t0, \t' + str(a[i]) + ', \t0,\t0, \t1\n')
            file_Rect.write( str(l) + ', \t0, \t' + str(s[0]) + ', \t0, \t0, \t'  + str(v[i]) + ', \t0,\t0, \t' + str(a[i]) + ', \t0,\t0\n')
            file_Rect.write( str(l) + ', \t' + str(-s[0]) + ', \t0, \t0, \t'  + str(-v[i]) + ', \t0,\t0, \t' + str(-a[i]) + ', \t0,\t0, \t0\n')
            file_Rect.write( str(l) + ', \t0, \t' + str(-s[0]) + ', \t0, \t0, \t'  + str(-v[i]) + ', \t0,\t0, \t' + str(-a[i]) + ', \t0,\t0\n')
            l += 1
            
    l = 0
    for i in range(w.shape[0]): 
        for j in range(n_wdh): 
            file_Rot.write( str(l) + ', \t0, \t0, \t' + str(phi[0]) + ', \t0, \t0, \t'  + str(w[i]) + ', \t0,\t0, \t' + str(dw[i]) + ', \t1\n')
            l += 1   # rotation 
        
if __name__ == '__main__':
    try:
#        if isinstance(sys.argv[1], str) and isinstance(sys.argv[2], str): 
#            datafile = sys.argv[1] + '/' + sys.argv[2]
        retVal = CreateMasterInput()
    except rospy.ROSInterruptException:
        pass