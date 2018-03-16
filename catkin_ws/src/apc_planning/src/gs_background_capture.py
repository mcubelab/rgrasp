#!/usr/bin/env python

import rospy, sys, os, tf, cv2
mcube_learning_path = os.environ['HOME'] + '/mcube_learning'
sys.path.append(mcube_learning_path)
import matplotlib.cm as cm
from helper.image_helper import convert_world2image, convert_image2world, translate_image, crop_gelsight, back_substraction, preprocess_image, get_center_of_mass, crop_contact, back_sub_process
from helper.helper import load_file
from grasping17 import check_collision
from cv_bridge import CvBridge, CvBridgeError
# from PIL import Image
from helper.visualize_CAM import plot_CAM
import numpy as np
import scipy
import sensor_msgs
import pdb
import ik
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray, String

if __name__ == '__main__':
    rospy.init_node('gs_background_capture', anonymous=True)
    print 'start node background'

    bridge_cv = CvBridge()
    image_topic_list = ["/rpi/gelsight/raw_image","/rpi/gelsight/raw_image2"]
    for it, topic in enumerate(image_topic_list):
        if rospy.get_param('have_robot'):
            print('hi')
            image_ros = rospy.wait_for_message(topic, sensor_msgs.msg.Image)
            print('hi')
            flip_cv_image = bridge_cv.imgmsg_to_cv2(image_ros, 'rgb8')
            if it == 0:
                flip_cv_image = cv2.flip(flip_cv_image,1)
            flip_cv_image = cv2.flip(cv_image,0)
            cv2.imwrite('/home/mcube/background_{}.png'.format(it),flip_cv_image)
        else:
            image_path = '/media/mcube/data/Dropbox (MIT)/images/gelsight_fingerprint.png'
            cv2.imwrite('/home/mcube/background_{}.png'.format(it), cv2.imread(image_path, 1))
