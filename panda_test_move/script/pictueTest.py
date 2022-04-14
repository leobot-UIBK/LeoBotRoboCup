#!/usr/bin/env python2
# -*- coding: utf-8 -*-
# based on the tutorial from
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# by Martin Sereinig, 10.06.2021

from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError





class image_converter:

  def __init__(self):
    # definition des ros publisher zum senden des veränderten Bildes, wird dann unter em Topic: image_topic_2 gepublished
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
    self.bridge = CvBridge()
    # definition des ros subscriber zum laden der Bilddaten (hier kommt der Name des Topics)
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
    # Umwandeln des Bildes von ROS zu OpenCV
      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
    except CvBridgeError as e:
      print(e)

    # Zeichnen eines Kreises
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
    # Rotieren des Bildes
    cv_image = cv2.transpose(cv_image)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    # Erstellen eines Binärbildes mit Threshold
    ret, thresh1= cv2.threshold(cv_image,127,255,cv2.THRESH_BINARY)
    cv2.imshow("Image binary", thresh1)
    cv2.waitKey(3)

    try:
        # Senden des erzeugten Bildes als ROS image topic
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, desired_encoding='bgr8'))
    except CvBridgeError as e:
      print(e)


#Wichtig, Main des ROS Knoten
def main(args):
  # ROS Knoten initialisieren  
  rospy.init_node('pictueTest', anonymous=True)
  # Initialisieren des Objektes ic der Klasse image_converter()
  ic = image_converter()
  try:
     #Wichtig damit die Main in endlosschleife ausgeführt wird, somit wird der Callback der Klasse immer gestartet sobald neue Daten im Topic des Subscribers vorliegen
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
