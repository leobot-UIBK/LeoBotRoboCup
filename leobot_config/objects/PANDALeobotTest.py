# -*- coding: utf-8 -*-
"""
Created on Thu May 27 11:38:55 2021
Testing Corke Robotics Toolbox in python
https://github.com/petercorke/spatialmath-python
https://github.com/petercorke/robotics-toolbox-python


@author: c8501077
"""


#standard import
import numpy as np
import matplotlib.pyplot as plt

#exudyne import
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *
#import exudyn.roboticsSpecial as roboticsSpecial
import roboticsSpecial
import manipulatorImport

import scipy.io


#corke robotics toolbox
import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import *



#%%

# Item stack1





phiz=[23, 0, -13.5, -21, 15, 0, -21, -21, 23, -64, 32.5 , 32.5, -64, 44, 44]  
phiy=[20, 35, 35,    35, 35, 35, 35,  35, 20,   0,    0,     0,   0,  0,  0]


rxyz=[]

for i in range(0,15):
    R1 =rotz(phiz[i]*np.pi/180)
    R2=roty(phiy[i]*np.pi/180) # @ roty(np.pi) @ rotz(np.pi)
    rxyz.append(list(RotationMatrix2RotXYZ(R1@R2).round(10)))
    
    
    
    
    
    #print ([rxyz[0],rxyz[1],rxyz[2]])
    #reexyz=RotationMatrix2RotXYZ(R1@R2@roty(np.pi)).round(4)


print(np.array(rxyz))



# # write data in a file.
# file1 = open("myfile.txt","w")
# L = ["This is Delhi \n","This is Paris \n","This is London \n"] 
  

# for element in rxyz:
#     file1.write(element + "\n")


# file1.close() #to change file access modes













#%%
# Quaternonen to Matrix (w,x,y,z)
R=EulerParameters2RotationMatrix([0,0,1,0])
rxyz=RotationMatrix2RotXYZ(R)

roty(np.pi)

#%%
# Stack left and front from panda_link0
offx=-0.2+0.04
offy=-0.1-0.15
offz=0.21

P0Stack=np.array([offx,offy,offz])

#1
objStack1_F20_20=P0Stack+[-0.35,0.045,0.045]
objStack1_S40_40=P0Stack+[-0.35,0.045,0.045]

#2
objStack1_Axis=P0Stack+[-0.36, 0.25,0.05]

#3
objStack2_Axis=P0Stack+[-0.315,0.365,0.05]

#4
objStack1_Motor=P0Stack+[-0.345,0.455,0.055]

#5
objStack3_Axis=P0Stack+[-0.24,0.145,0.05]

#6
objStack1_R20=P0Stack+[-0.20,0.265,0.04]

#7
objStack2_R20=P0Stack+[-0.23,0.385,0.04]

#8
objStack2_Motor=P0Stack+[-0.235,0.455,0.055]

#9
objStack2_F20_20=P0Stack+[-0.13,0.085,0.045]

#10
objStack1_Bearing_Box=P0Stack+[-0.15, 0.36, 0.0325]

#11
objStack1_M20=P0Stack+[-0.1,0.415,0.0275]


#12
objStack1_M30=P0Stack+[-0.15,0.445,0.0315]


#13
objStack2_Bearing_Box=P0Stack+[-0.08,0.33,0.0325]


#14
objStack1_Distance_Tube=P0Stack+[-0.035,0.44,0.025]


#15
objStack2_M20=P0Stack+[-0.075,0.48,0.0275]
















































