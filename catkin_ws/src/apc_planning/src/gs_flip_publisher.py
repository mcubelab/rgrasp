import os, errno, sys
import numpy as np
import rospy
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge, CvBridgeError



def flip_image(data):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(flip_cv_image,0)
    #publish flipped image
    flip_image_pub = rospy.Publisher("rpi/gelsight/flip_raw_image",sensor_msgs.msg.Image)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(flip_cv_image, "bgr8"))


def listener():
    rospy.init_node('flip_image', anonymous=True)
    image_sub = rospy.Subscriber("rpi/gelsight/raw_image", sensor_msgs.msg.Image, flip_image)
    rospy.spin()

if __name__ == '__main__':
    listener()
