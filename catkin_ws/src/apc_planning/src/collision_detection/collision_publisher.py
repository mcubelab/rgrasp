#!/usr/bin/env python

import rospy
import sys
import rospy
import numpy as np
from numpy import linalg as la
from manual_fit.srv import *
from collisionHelper import  getBinPoints, getFingerPoints, getWristPoints, getToteSections
import tf
import tf.transformations as tfm
import os
from visualization_msgs.msg import MarkerArray
import time
from ik.helper import deleteMarkers, plotPickPoints, plotBoxCorners, get_tcp_pose, matrix_from_xyzquat
import gripper
import std_msgs.msg

# To test the function
if __name__=='__main__':
    #~ Frank & Nikhil:hack to get proper vision format
    rospy.init_node('collision_publisher', anonymous=True)
    
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    
    viz_pub                = rospy.Publisher('/collision_visualization_marker_array', MarkerArray, queue_size=10)
    experiment_comments_pub=rospy.Publisher('/exp_comments', std_msgs.msg.String, queue_size = 10)
    pub = rospy.Publisher('topic_name', std_msgs.msg.String, queue_size=10)
    rospy.sleep(1)
    
    ###################################
    r = rospy.Rate(10)
    counter = 0
    
    experiment_description = "Version: 1.0, Objects: Scotch Brite Sponges, Comments: Only grasping is recorded (not placing), Passive vision pointcloud and rgb-d are not recorded."
    
    while True:
        comments_msgs = std_msgs.msg.String()
        comments_msgs.data = experiment_description
        experiment_comments_pub.publish(comments_msgs)
        
#        ##########
#        ## Bins ##
#        ##########
#        for i in range(0,3):
#            bin_points = getBinPoints(i, listener, br)
#            my_rgb = (0,0,1,1)
#            plotBoxCorners(bin_points,viz_pub, my_rgb, namespace = 'corner_points'+str(i))
#            
#        #############
#        ## Gripper ##
#        #############
#        finger_opening = gripper.getGripperopening()
#        tcp_pose = get_tcp_pose(listener)
#        tcp_pos = tcp_pose[0:3]
#        tcp_quaternion = tcp_pose[3:7]
#        #~ ******************* Tcp Frame   *******************
#        tcp_pose_tfm_list=matrix_from_xyzquat(tcp_pos,tcp_quaternion)
#        tcp_pose_tfm=np.array(tcp_pose_tfm_list)
#        tcp_pose_orient=tcp_pose_tfm[0:3,0:3]
#        #Normalized axes of the shelf frame
#        tcp_X=tcp_pose_orient[:,0]/la.norm(tcp_pose_orient[:,0])
#        tcp_Y=tcp_pose_orient[:,1]/la.norm(tcp_pose_orient[:,1])
#        tcp_Z=tcp_pose_orient[:,2]/la.norm(tcp_pose_orient[:,2])
#        hand_orient_norm = np.vstack([tcp_X,tcp_Y,tcp_Z])
#        hand_orient_norm=hand_orient_norm.transpose()
#        fingerPoints= getFingerPoints(finger_opening, tcp_pos, hand_orient_norm, False)
#        wristPoints = getWristPoints(tcp_pos, hand_orient_norm, False)
##        tubePoints = getSuctionTubePoints(tcp_pos, hand_orient_norm, False)
##        totePointsList = getToteSections(listener = listener, br=br)
#        
#        my_rgb = (0,1,0,1)
#        plotBoxCorners(fingerPoints,viz_pub, my_rgb, namespace = 'fingerPoints'+str(i))
#        plotBoxCorners(wristPoints,viz_pub, my_rgb, namespace = 'wristPoints'+str(i))
#        plotBoxCorners(tubePoints,viz_pub, my_rgb, namespace = 'tubePoints'+str(i))
        #~ plotBoxCorners(totePointsList[0],viz_pub, my_rgb, namespace = 'totePointsList'+str(i))
        #~ plotBoxCorners(totePointsList[1],viz_pub, my_rgb, namespace = 'totePointsList'+str(i))
        #~ plotBoxCorners(totePointsList[2],viz_pub, my_rgb, namespace = 'totePointsList'+str(i))
        #~ plotBoxCorners(totePointsList[3],viz_pub, my_rgb, namespace = 'totePointsList0'+str(i))
        #~ plotBoxCorners(totePointsList[4],viz_pub, my_rgb, namespace = 'totePointsList1'+str(i))
        #~ plotBoxCorners(totePointsList[5],viz_pub, my_rgb, namespace = 'totePointsList2'+str(i))
        #~ plotBoxCorners(totePointsList[6],viz_pub, my_rgb, namespace = 'totePointsList3'+str(i))
        counter+=1

            
        r.sleep()
    ##################################
    
    #~ deleteMarkers(viz_pub,'object_bounding_box_for_collision')
    #~ deleteMarkers(viz_pub,'object_bounding_box_for_collision2')
