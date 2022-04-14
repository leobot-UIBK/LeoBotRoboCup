#!/usr/bin/env python

from __future__ import print_function

import os
import cv2
import numpy as np
import rospy
import rospkg
import json
import math
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs import point_cloud2 as pc2
import tf.transformations
import tf

from custom_msgs.msg import Obstacles
from custom_msgs.msg import Form
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

rospack = rospkg.RosPack()
package_path = rospack.get_path('robocup_tape_detection')

TEST = False

def color_histogram_and_backproj(target, roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsvt = cv2.cvtColor(target, cv2.COLOR_BGR2HSV)
    #cv2.imshow('hsv', hsv)
    #cv2.imshow('hsvt', hsvt)
    
    M = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])

    cv2.normalize(M, M, 0, 255, cv2.NORM_MINMAX)
    B = cv2.calcBackProject([hsvt], [0, 1], M, [0, 180, 0, 256], 1)

    _, thresh = cv2.threshold(B, 0, 255, cv2.THRESH_BINARY)

    thresh = cv2.merge((thresh, thresh, thresh))

    #cv2.imshow('thresh', thresh)
    #cv2.waitKey(0)
    

    return thresh    

def normalize_thresh(thresh):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    image = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    for i in range(10):
        image = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    for i in range(10):
        image = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)

    image = cv2.GaussianBlur(image, (5, 5), 0)

    return image


def get_contours(image):
    new_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    _, thresh = cv2.threshold(new_image, 180, 255, cv2.THRESH_BINARY)

    _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  
    return contours

def get_max_boundingbox(contours):
    boxes = []
    # Take the largest countor
    max_area = 0
    max_ind = 0
    max_contour = None
    for cnt in contours:
        # Calculate the area of each contour
        area = cv2.contourArea(cnt)
 
        # Ignore contours that are too small or too large
        if area < 10 or 100000 < area:
            continue

        if area > max_area:
            max_contour = cnt
            max_area = area

    print('max_contour')
    x,y,w,h = cv2.boundingRect(max_contour)

    if w + h > 10:
        rect = cv2.minAreaRect(max_contour)
        box = cv2.boxPoints(rect)

        box = np.int0(box)
        boxes.append(box)
    
    return boxes

def get_boundingboxes(contours, min_area=150):
    boxes = []
    for cnt in contours:
        #M = cv2.moments(cnt)

        x,y,w,h = cv2.boundingRect(cnt)

        if w + h > 150:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)

            box = np.int0(box)
            boxes.append(box)
    
    return boxes

def calculate_boundingbox_of_roi(image, hist):
    target = color_histogram_and_backproj(image, hist)
    cv_img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define limits of yellow HSV values
    yellow_lower = np.array([16, 60, 100])
    yellow_upper = np.array([45, 255, 255])

    # Filter the image and get the mask
    target2 = cv2.inRange(cv_img_hsv, yellow_lower, yellow_upper)


    target2 = normalize_thresh(target2)
    #cv2.imshow('roi', image)
    #cv2.imshow('target2', target2)
    #cv2.waitKey(0)
    #contours = get_contours(target2)
    _, contours, _ = cv2.findContours(target2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    boxes = get_boundingboxes(contours)

    if len(boxes) > 0:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 45, 255, cv2.THRESH_BINARY_INV)
        thresh = normalize_thresh(thresh)
        # Filter the image and get the mask
        #target2 = cv2.inRange(cv_img_hsv, black_lower, black_upper)


        #target2 = normalize_thresh(target2)
        #cv2.imshow('thresh', thresh)
        print('Yellow found')
        #cv2.waitKey(0)
        #contours = get_contours(target2)
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        boxes = get_boundingboxes(contours, min_area=30)

    return boxes, target

def calculate_lines_canny(image):
    edges = cv2.Canny(image, 150, 240) #, L2gradient=True)
    #cv2.imshow('edge', edges)
    #cv2.waitKey(0)
    _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    boxes = get_boundingboxes(contours)
    return boxes

def resize_image(img, scale_percent): 
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    return cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

def array_to_point(rawpoint):
    point = Point()
    point.x = rawpoint[0]
    point.y = rawpoint[1]
    point.z = rawpoint[2]
    return point

def create_line(pointlist):
    point1 = array_to_point(pointlist[0])
    point2 = array_to_point(pointlist[1])
    newform = Form()
    newform.form = [point1, point2]

    line = Obstacles()
    line.list = [newform]
    return line


def tape_detection_test():
    rospy.init_node('tape_detection_test')
    pub = rospy.Publisher('/virtual_costamp_layer/obsctacles', Obstacles, queue_size=2)
    tf_listener_ = tf.TransformListener()

    # roi is the object or region of object we need to find
    roi_yb = cv2.imread(package_path + "/data/tape_yellow_black_hist.jpeg")
    while not rospy.is_shutdown():
        if TEST:
            points3D = np.random.rand(1000, 1500, 3) # Random point cloud
            # target is the image we search in, testing here
            cv_image = cv2.imread(package_path + "/data/tape_yellow_black_new.jpeg")

            #cv2.imshow('image', cv_image)
            #cv2.imshow('image_hsv', cv_img_hsv)
            #cv2.imshow('mask', mask)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
        else:
            # Get point cloud
            point_cloud_msg = rospy.wait_for_message("/camera_base/depth/color/points", PointCloud2, timeout=None)
            # Get current image and convert to cv2
            image_msg = rospy.wait_for_message("/camera_base/color/image_raw", Image, timeout=None)
            cv_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

            # Point cloud to numpy array (assume ordered point cloud, set ordered_pc)
            points3D = np.array(list(pc2.read_points(point_cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))).reshape((image_msg.height,image_msg.width,3))

            contours = calculate_lines_canny(cv_image)
            #contours, _ = calculate_boundingbox_of_roi(cv_image, roi_yb)

        for contour in contours:
            # Color based detection
            x_min = max(0, contour[:,0].min())
            x_max = max(0, contour[:,0].max())
            y_min = max(0, contour[:,1].min())
            y_max = max(0, contour[:,1].max())

               # Apply template Matching
            template = cv_image[y_min:y_max, x_min:x_max, :]
            #res = calculate_lines_canny(template)
            res, _ = calculate_boundingbox_of_roi(template, roi_yb)
            #res = cv2.matchTemplate(template, roi_yb, cv2.TM_CCOEFF_NORMED)
            if len(res) > 0:
                print('Tape found!')
                box = contour
                # Get segment corners
                if np.linalg.norm(box[1,:]-box[0,:]) <= np.linalg.norm(box[3,:]-box[0,:]):
                    c1 = [int(math.floor(box[0:2, 0].mean())), int(math.floor(box[0:2, 1].mean()))]
                    c2 = [int(math.floor(box[2:4, 0].mean())), int(math.floor(box[2:4, 1].mean()))]
                else:
                    c1 = (int(math.floor(box[[0,3],0].mean())), int(math.floor(box[[0,3],1].mean())))
                    c2 = (int(math.floor(box[[1,2],0].mean())), int(math.floor(box[[1,2],1].mean())))
                if TEST:
                    cv2.drawContours(cv_image, [contour], 0, (0, 0, 255), 14)
                    #cv_image = cv2.line(cv_image, c1, c2, (255, 0, 0), 14)

                # Build ROS message and transform it into base_link frame
                c1 = list(c1)
                c2 = list(c2)
                if c2[1] >= 720:
                    c2[1] = 719
                if c2[0] >= 1280:
                    c2[0] = 1279
                if c1[1] >= 720:
                    c1[1] = 719
                if c1[0] >= 1280:
                    c1[0] = 1279

                p1 = points3D[c1[1], c1[0], :]
                p2 = points3D[c2[1], c2[0], :]


                #p1 = Transform().make_transform(p1, 'camera_base_color_optical_frame', 'map')
                #p2 = Transform().make_transform(p2, 'camera_base_color_optical_frame' 'map')
                transform = get_transformation('camera_base_color_optical_frame', 'map')
                #p_frame1 = transform_pose_stamped(create_poseStamped(p1), "map", tf_listener_)
                #p_frame2 = transform_pose_stamped(create_poseStamped(p2), "map", tf_listener_)

                #point_new_frame1= [p_frame1.pose.position.x, p_frame1.pose.position.y, p_frame1.pose.position.z]
                #point_new_frame2 = [p_frame2.pose.position.x, p_frame2.pose.position.y, p_frame2.pose.position.z]

                #poses = PoseStamped()
                #print(p1, transform[1])
                mat = tf.transformations.quaternion_matrix(transform[1])[:3, :3]
                p1n = np.matmul(mat, np.array(p1))
                p2n = np.matmul(mat, np.array(p2))

                p1n = p1n + transform[0]
                p2n = p2n + transform[0]
                p1n[2] = 0.0
                p2n[2] = 0.0
                print(p1n)

                pub.publish(create_line([list(p1n), list(p2n)]))
                rospy.sleep(0.1)


        if TEST:
            cv_image = resize_image(cv_image, 40)
            cv2.imshow('Tape detection YB', cv_image)
            cv2.waitKey(5)

    cv2.destroyAllWindows()



# transforms from current frame to new frame
def transform_pose_stamped(pose, new_frame, tf_listener_):

    t = tf_listener_.getLatestCommonTime("camera_base_color_optical_frame", "map")
    #tf_listener_.waitForTransform("map", "camera_base_color_optical_frame", t, timeout=rospy.Duration(4))

    p_frame2 = tf_listener_.transformPose(new_frame, pose)
    # print "Position of the fingertip in the robot base:" # print for debugging.
    # print p_frame2
    return p_frame2


def create_poseStamped(raw_point):
    new_point = PoseStamped()
    new_point.header.frame_id = "camera_base_color_optical_frame"
    #new_point.header.stamp = rospy.Time.now()
    new_point.pose.position.x = raw_point[0]
    new_point.pose.position.y = raw_point[1]
    new_point.pose.position.z = raw_point[2]
    new_point.pose.orientation.x = 0.
    new_point.pose.orientation.y = 0.
    new_point.pose.orientation.z = 0.
    new_point.pose.orientation.w = 1.
    return(new_point)

'''

class Transform:
    def __init__(self, *args):
        self.tf_listener_ = TransformListener()

    def make_transform(self,point, frame1, frame2):
        if self.tf.frameExists(frame1) and self.tf.frameExists(frame2):
            t = self.tf_listener_.getLatestCommonTime(frame1, frame2)
        pose = create_poseStamped(point)
            p_frame2 = self.tf_listener_.transformPose(frame2, pose)
        point_new_frame = [p_frame2.pose.position.x, p_frame2.pose.position.y, p_frame2.pose.position.z]
            print("Position of the fingertip in the robot base:")
        return(point_new_frame)

# transforms from frame1 to frame2
def transform_pose(pose, frame1, frame2):
	tf_listener_ = tf.TransformListener()
	t = tf_listener_.getLatestCommonTime(frame1, frame2)
	p1 = create_poseStamped(pose)
	p_frame2 = tf_listener_.transformPose(frame2, p1)

	point_new_frame = [p_frame2.pose.position.x, p_frame2.pose.position.y, p_frame2.pose.position.z]
	print("Position of the tape_point in the map frame:")
	print(point_new_frame)
	return(point_new_frame)
'''


def get_transformation(frame1,frame2):
    listener = tf.TransformListener()
    listener.waitForTransform(frame2, frame1, rospy.Time(0), timeout=rospy.Duration(4))
    (trans,rot) = listener.lookupTransform(frame2, frame1, rospy.Time(0))
    return(trans, rot)



if __name__ == "__main__":
    tape_detection_test()
