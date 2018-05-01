#!/usr/bin/python

import random, time, datetime, json, optparse, rospy, copy, yaml, cv2, math, subprocess
import tf
import ik.visualize_helper
import numpy as np
import os
from ik.marker_helper import createDeleteAllMarker
from grasp_data_recorder import GraspDataRecorder
import signal
import sys
import spatula, gripper
try:
    import passive_vision.srv
except:
    print('FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL')

import sys
sys.path.append(os.environ['CODE_BASE']+'/catkin_ws/src/weight_sensor/src')
sys.path.append(os.environ['HOME'] + '/mcube_learning')
from models.models import two_gelsight_model
import ws_prob
import goToHome
from grasping17 import place, grasp, retrieve, release_safe, grasp_correction
from control_policy import controlPolicy
from ik.helper import fake_bbox_info_1, Timer, vision_transform_precise_placing_with_visualization, get_params_yaml
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from gelsight import calibrate_gelsight
import sensor_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
import pdb
from helper.image_helper import convert_world2image, convert_image2world, translate_image, crop_gelsight, back_substraction, preprocess_image, get_center_of_mass, crop_contact, back_sub_process
import scipy
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

class TaskPlanner(object):
    def __init__(self, opt):
        # Configuration
        self.virtual = opt.virtual
        self.goHomeSlow = False
        # ROS setup
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #Class Publishers
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.proposal_viz_array_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
        self.bbox_info = fake_bbox_info_1(self.listener)#Give bounding box to the object
        rospy.sleep(0.5)

    def run_experiments(self):
        
        # Start execution
        goToHome.goToARC(slowDown=self.goHomeSlow) # 1. Initialize robot state
        # Place approx. at the center bin
        binId = 0
        drop_pose = get_params_yaml('bin{}_pose'.format(binId))
        drop_pose[1] = drop_pose[1] + 0.03
        # Place object using grasping
        
        self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info,viz_pub=self.viz_array_pub,listener=self.listener)
        place(listener=self.listener, br=self.br,isExecute=True, binId=binId,  withPause=False,rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose, viz_pub=self.viz_array_pub, is_drop = False,
                         isHack = True, guard_speed = (100,100))

        

if __name__ == '__main__':
    
    rospy.init_node('Planner')
    
    parser = optparse.OptionParser()
    parser.add_option('-v', '--virtual', action='store_true', dest='virtual',
        help='To pause or not', default=False)
    
    (opt, args) = parser.parse_args()
    p = TaskPlanner(opt)
    p.run_experiments()
