#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge

def flip_image(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(flip_cv_image,0)
    #publish flipped image
    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(flip_cv_image, "bgr8"))


def listener():
    image_sub = rospy.Subscriber("rpi/gelsight/raw_image", sensor_msgs.msg.Image, flip_image, "rpi/gelsight/flip_raw_image")
    image_sub2 = rospy.Subscriber("rpi/gelsight/raw_image2", sensor_msgs.msg.Image, flip_image, "rpi/gelsight/flip_raw_image2")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('gs_flip_publisher', anonymous=True)
    print 'start node'
    listener()
