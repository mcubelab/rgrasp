#!/usr/bin/env python

import json
import os
from pprint import pprint
from ik.roshelper import coordinateFrameTransform
from ik.helper import getBinMouthAndFloor
from ik.roshelper import pose2list
from ik.helper import get_bin_cnstr
import sys
import rospy
import tf
import numpy as np

from ik.roshelper import pubFrame
from ik.roshelper import poseTransform
from ik.roshelper import ROS_Wait_For_Msg
from pr_apriltags.msg import AprilTagDetections
from ik.roshelper import pose2list
from ik.helper import matrix_from_xyzquat
from ik.helper import pause
import pdb
import tf.transformations as tfm

import math


def main(argv=None):
    if argv is None:
        argv = sys.argv
    rospy.init_node('calib', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(1)
    
    calibtag = 'link6tag1'
    if len(argv) == 2:
        calibtag = argv[1]
    
    #filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/camera_extrinsic_calib_data/camera_extrinsic_calib_data.json'
    if calibtag == 'link2tag':
        tag_link_transform = [0, 0.7, -0.2145-0.005, -math.pi, 0, 0]  
        link_frame_id = 'link_2'
    elif calibtag == 'realsense1':
        print 'move to q =', (np.array([ 0.0, -23.02, 34.73, 154.93, -25.15, 0.0]) / 180 * math.pi).tolist()
        
        r = 0.1923/2
        tag_link_transform = [0.4033-0.230 -r, -1.121/2 + 0.044 + r, 0.005, 0, 0, 0]  
        link_frame_id = 'base_link'
    elif calibtag == 'link6tag1':
        #print 'move to q =', (np.array([9.93,39.37,-30.63,60.64,-12.12,-61.79]) / 180 * math.pi).tolist()
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, math.pi/2]  
        link_frame_id = 'link_6'
    elif calibtag == 'link6tag2':
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, 0]  
        link_frame_id = 'link_6'
    elif calibtag == 'link6tag3':
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, -math.pi/2]  
        link_frame_id = 'link_6'
    elif calibtag == 'link6tag4':
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, math.pi]  
        link_frame_id = 'link_6'
        
    # visualize it
    pubFrame(br, pose=tag_link_transform, frame_id='tag', parent_frame_id=link_frame_id, npub=10)
    
    
    tag_map_transform = poseTransform(tag_link_transform, link_frame_id, 'map', listener)
    
    apriltag_topic = '/pr_apriltags/detections' 
    print 'Wait for apriltag detection'
    tag_camera_transforms = []
    for i in xrange(5):
        while True:
            tagdetect = ROS_Wait_For_Msg(apriltag_topic, AprilTagDetections).getmsg() 
            if len(tagdetect.detections) == 1:
                tag_camera_transform = pose2list(tagdetect.detections[0].pose)
                tag_camera_transforms.append(tag_camera_transform)
                break
    
    tag_camera_transform = np.average(np.array(tag_camera_transforms), axis = 0)
    
    tag_camera_tfm_mat = matrix_from_xyzquat(tag_camera_transform) 
    tag_map_tfm_mat = matrix_from_xyzquat(tag_map_transform) 
    
    camera_map_tfm_mat = np.dot(tag_map_tfm_mat, np.linalg.inv(tag_camera_tfm_mat))
    
    camera_map_pose = tfm.translation_from_matrix(camera_map_tfm_mat).tolist() + tfm.quaternion_from_matrix(camera_map_tfm_mat).tolist()
    

    link6tag = ['link6tag1', 'link6tag2', 'link6tag3', 'link6tag4']
    if calibtag in link6tag:
        print camera_map_pose
        pubFrame(br, pose=camera_map_pose, frame_id='new_kinect_pose', parent_frame_id='map', npub=10)
    elif calibtag == 'realsense1':
        realsense_link5_pose = poseTransform(camera_map_pose, link_frame_id, 'link_5', listener)
        pubFrame(br, pose=realsense_link5_pose, frame_id='new_realsense_pose', parent_frame_id='link_5', npub=10)
        
        print realsense_link5_pose

    #april_tag_poses_robot = []
    
    #april_tag_pose_camera

if __name__=='__main__':
    sys.exit(main())
    
