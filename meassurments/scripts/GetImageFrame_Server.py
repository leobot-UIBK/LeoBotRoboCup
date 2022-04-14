#!/usr/bin/env python
# from tutorial opencv-bridge 

from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Trigger, TriggerResponse

import os

class image_converter:
    def __init__(self, directory, filename, ImageTopic, ImageTopicDepth, i_start):
        self.directory = directory
        self.filename = filename
        self.i_frame = i_start
        self.bridge = CvBridge()
        self.image_Server = rospy.Service('/get_image_frame', Trigger, self.getImage)
        self.image_sub = rospy.Subscriber(ImageTopic, Image,self.callback)
        self.image_subDepth = rospy.Subscriber(ImageTopicDepth, Image,self.callbackDepth)
        
    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callbackDepth(self,data):
        try:
            self.cv_imageDepth = self.bridge.imgmsg_to_cv2(data, "16UC1")
        except CvBridgeError as e:
            print(e)

        #(rows,cols,channels) = self.cv_image.shape
        #if cols > 60 and rows > 60 :
            #cv2.circle(self.cv_image, (50,50), 10, 255)
        #cv2.imshow("Image window", self.cv_image)
        #cv2.waitKey(3)
            
    def getImage(self, Trigger_srv):
        str_num = str(0) * (5-(len(str(self.i_frame))))
        FilePath = self.directory + '/' + self.filename + '_' + str_num + str(self.i_frame) + '.png' 
        FilePathDepth = self.directory + '/' + self.filename + '_' + str_num + str(self.i_frame) + '_depth' +'.png' 
        cv2.imwrite(FilePath, self.cv_image)
        cv2.imwrite(FilePathDepth, self.cv_imageDepth)
        # get next image frame from topic
        self.i_frame +=1
        return TriggerResponse(
            success=True,
            message="Frame " + 'saved to ' + FilePath + '.'
        )
def main(args):
#    i = 0
#    for arg in args: 
#        print('Argument ' + str(i) + ':')
#        print(arg)
#        i+=1
    directory = args[1]
    filename = args[2] # 'testimage'
    ImageTopic = args[3] #'/camera/color/image_raw'
    ImageTopicDepth = args[4] #'/camera/depth/image_rect_raw'
    try: 
        os.makedirs(directory)
    except OSError: 
        print("Created Directory %s." % directory)
    else: 
        print("Directory is created.")
    list_items = os.listdir(directory) # dir is your directory path
    i_start = (len(list_items))/2
    print('There are '+str(i_start) + ' files in the directory ' + str(directory) + '.')
    ic = image_converter(directory, filename, ImageTopic, ImageTopicDepth, i_start)
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()    
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
