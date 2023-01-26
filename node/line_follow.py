#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image

## init camera subscriber
bridge = CvBridge()
## create callback which processes camera input
# searches for line using algorithm developed in lab 2
# publishes a twist message with linear and angular velocities
def image_callback(msg):
    #convert image to opencv format
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    width = cv_image.shape[1]
    height = cv_image.shape[0]
    #process image with lab 2 algorithm
    (x, y) = find_line(cv_image)
    cv2.circle(cv_image, (x, y), 5, (0, 0, 255), -1)
    #publish velocity commands to robot
    twist = Twist()
    twist.linear.x = 0.15
    twist.angular.z = -(x - width/2) / 150
    pub.publish(twist)
    cv2.imshow('image', cv_image)
    cv2.waitKey(1)

## algorithm from lab 2 to find line in image
# returns x,y coordinates of line
# @param frame: image to search for line
def find_line(frame):
    width = frame.shape[1]
    height = frame.shape[0]
    #how many pixels from the bottom to scan for line
    scan_height = 50
    line_threshold = 160
    #convert to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    scan_line = ((255 - gray_frame[:][height - scan_height]) > line_threshold)
    avg = np.sum(np.arange(width) * scan_line) / (np.sum(np.ones(width) * scan_line) + 0.1)
    x_line = ((int)(avg))
    return (x_line, height - scan_height)

## init velocity publisher
rospy.init_node('topic_publisher', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2) # 10hz
sub = rospy.Subscriber('/robot/camera1/image_raw', Image, image_callback)

while not rospy.is_shutdown():
    
    rate.sleep()
