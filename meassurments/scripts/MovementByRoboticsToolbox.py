# -*- coding: utf-8 -*-
"""
Created on Thu Apr  7 08:48:13 2022

@author: c8501100
"""

import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import *
import numpy as np 

# for debugging only: 
from exudyn.rigidBodyUtilities import *
import time 

robot = rtb.models.DH.Panda()


TCurrent = robot.fkine(robot.qz)

# robot.plot(robot.qz)

def CalcAngRelMovement(qCurrent, tRel, phiRel): 
    TCurrent = robot.fkine(qCurrent)
    TNew = TCurrent @ SE3(transl(tRel)) @ SE3(rpy2tr(np.flip(phiRel), 'xyz')) # rpy2tr uses flipped angle inputs!! 
    qNew = robot.ikine_LMS(TNew)
    return qNew, TNew

t1 = time.time()
q = [robot.qz]
tRel = [0.1 * 0,0,0]
phiRel = [0,np.pi/100,0]
cntFailed = 0

for i in range(100): 
    qNew, TCurrent = CalcAngRelMovement(q[-1], tRel, phiRel)
    print('Angles: {}'.format(RotationMatrix2RotXYZ(TCurrent.R)))
    q += [qNew.q.tolist()]
    if not(qNew.success): 
        print('fail')
        cntFailed += not(qNew.success)
print('time elapsed: {}'.format(time.time() - t1))
robot.plot(np.array(q))
