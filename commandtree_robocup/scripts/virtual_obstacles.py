#! /usr/bin/env python

import rospy
from custom_msgs.msg import Obstacles
from custom_msgs.msg import Form
from geometry_msgs.msg import Point
import numpy as np
import time

'''
This script adds virtual obstacles to the local_costmap. It is possible to add a point,line or polygon (box).
Type of obstacle depends on the number of points in the obstacle variable.
'''

class NewPoint:
    ''' this class creates a gemoetry_msgs/Point message'''
    def create_virtual_obstacle(self, rawpoint):
        self.point = Point()
        self.point.x = rawpoint[0]
        self.point.y = rawpoint[1]
        self.point.z = rawpoint[2]
        return(self.point)

def create_point(point):
    point = NewPoint().create_virtual_obstacle(point)
    newform = Form()
    newform.form = [point]

    point = Obstacles()
    point.list = [newform]
    return(point)

def create_line(pointlist):
    point1 = NewPoint().create_virtual_obstacle(pointlist[0])
    point2 = NewPoint().create_virtual_obstacle(pointlist[1])
    newform = Form()
    newform.form = [point1, point2]

    line = Obstacles()
    line.list = [newform]
    return(line)

def create_box(pointlist):
    point1 = NewPoint().create_virtual_obstacle(pointlist[0])
    point2 = NewPoint().create_virtual_obstacle(pointlist[1])
    point3 = NewPoint().create_virtual_obstacle(pointlist[2])
    point4 = NewPoint().create_virtual_obstacle(pointlist[3])

    newform = Form()
    newform.form = [point1, point2, point3, point4]

    box = Obstacles()
    box.list = [newform]
    return(box)

if __name__=="__main__":
    rospy.init_node('virt_obstacle_publisher')
    time.sleep(1)
    testpoint1 = [1.0, 0.5, 0.0]
    testpoint2 = [1.0, 0.0, 0.0]
    testpoint3 = [1.5, 0.0, 0.0]
    testpoint4 = [1.5, 0.5, 0.0]

    obstacle = [testpoint1, testpoint2, testpoint3, testpoint4]

    if len(obstacle) == 1 :
        obstacle = create_point(obstacle)
    elif len(obstacle) == 2 :
        obstacle = create_line(obstacle)
    elif len(obstacle) >= 3 :
        obstacle = create_box(obstacle)

    pub = rospy.Publisher('/virtual_costamp_layer/obsctacles', Obstacles)
    
    #somehow it inly works when teh mesage is published three times with 0.5 sleep
    for i in range(1,3):
        pub.publish(obstacle)
        rospy.sleep(0.5)
        i = i +1
    rospy.loginfo('Published the new virtual obstacle')
