#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge
sys.path.append(os.environ['CODE_BASE']+'/catkin_ws/src/weight_sensor/src')
sys.path.append(os.environ['HOME'] + '/mcube_learning')
from helper.image_helper import get_center_of_mass, crop_contact
def flip_image(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(flip_cv_image,0)
    back_cv_image = cv2.imread('/home/mcube/background_0.png', 1)
    flip_cv_image = crop_contact(back_cv_image, flip_cv_image, gel_id = 1, is_zeros=True)
    COM_pos = get_center_of_mass(flip_cv_image)
    COM_pos = np.nan_to_num(COM_pos)
    print(COM_pos)
    cv2.rectangle(flip_cv_image, (int(COM_pos[1]-10),int(COM_pos[0]-10)),
                (int(COM_pos[1]+10), int(COM_pos[0]+10)),(0, 0, 255), -1)
    #publish flipped image
    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(flip_cv_image, "bgr8"))

def flip_image2(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(cv_image,0)

    back_cv_image = cv2.imread('/home/mcube/background_1.png', 1)
    flip_cv_image = crop_contact(back_cv_image, flip_cv_image, gel_id = 2, is_zeros=True)
    COM_pos = get_center_of_mass(flip_cv_image)
    COM_pos = np.nan_to_num(COM_pos)
    print(COM_pos)
    cv2.rectangle(flip_cv_image, (int(COM_pos[1]-10),int(COM_pos[0]-10)),
                (int(COM_pos[1]+10), int(COM_pos[0]+10)),(0, 0, 255), -1)
    #publish flipped image
    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(flip_cv_image, "bgr8"))

def listener():
    image_sub = rospy.Subscriber("rpi/gelsight/raw_image", sensor_msgs.msg.Image, flip_image, "rpi/gelsight/flip_raw_image")
    image_sub2 = rospy.Subscriber("rpi/gelsight/raw_image2", sensor_msgs.msg.Image, flip_image2, "rpi/gelsight/flip_raw_image2")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('gs_flip_publisher', anonymous=True)
    print 'start node'
    listener()
