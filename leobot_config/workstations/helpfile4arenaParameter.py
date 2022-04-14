#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Jun 19 21:07:16 2021

@author: robo
"""

# help calculations for arena parameter 

#standard import
import numpy as np

#%%


deskXsize=0.5
deskYsize=0.8
robotXsize=0.7
robotYsize=0.5

#####
YoffsetWorldPP01=0.666
PP01X= robotXsize/2+deskXsize-0.1
PP01Y= YoffsetWorldPP01+deskYsize/2.0


#####
WS01X= PP01X+0.1+1.8-0.5+0.1
WS01Y= PP01Y+0.510+deskYsize/2+deskYsize/2

####
WS02X= WS01X+0.3
WS02Y= WS01Y


####
WS03X= WS02X+0.1+1.6+2.0-deskYsize/2
WS03Y= WS02Y+deskYsize/2-deskXsize+0.1


####
WS04X= WS03X
WS04Y=WS03Y+0.4-2.5+deskXsize-0.1


####
WS05X=WS04X
WS05Y=WS04Y-0.3

####
WS06X=WS05X-deskYsize/2-1.69-0.1
WS06Y=WS05Y-0.1-2.085+1.2+deskYsize/2



####
WS07X=WS06X-0.4-0.02-0.4
WS07Y=WS06Y+deskYsize/2+0.43-deskYsize/2


####
WS08X=WS02X+0.1+1.6-0.4
WS08Y=WS02Y

####
WS09X=WS08X+0.3
WS09Y=WS08Y



####
SH01X= WS07X-0.1-0.65-deskYsize/2
SH01Y= WS07Y-deskYsize/2-1.15-0.1

print('WS01=  ', [WS01X,WS01Y])
print('WS02=  ', [WS02X,WS02Y])
print('WS03=  ', [WS03X,WS03Y])
print('WS04=  ', [WS04X,WS04Y])
print('WS05=  ', [WS05X,WS05Y])
print('WS06=  ', [WS06X,WS06Y])
print('WS07=  ', [WS07X,WS07Y])
print('WS08=  ', [WS08X,WS08Y])
print('WS09=  ', [WS09X,WS09Y])
print('SH01=  ', [SH01X,SH01Y])
print('PP01=  ', [PP01X,PP01Y])

















