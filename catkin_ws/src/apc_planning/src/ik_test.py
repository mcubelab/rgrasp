#!/usr/bin/env python

import rospy
import sys
import rospy
import numpy as np
from numpy import linalg as la
from manual_fit.srv import *
from collision_detection.collisionHelper import collisionCheck, getBinPoints
import tf
import tf.transformations as tfm
import os
from visualization_msgs.msg import MarkerArray
import time
from ik.helper import deleteMarkers, plotPickPoints, plotBoxCorners
from ik.ik import generatePlan

if __name__=='__main__':

    rospy.init_node('ik_test', anonymous=True) 
    
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    viz_pub = rospy.Publisher('/ik_visualization_marker_array', MarkerArray, queue_size=10)
    
    rospy.loginfo('[IK Test] Begin script')
    
    #~initialize values
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    gripperOri = [0,1,0,0]
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    
    ###########################
    ## Compute ik loop x,y,z ##
    ###########################
    point = []
    plans = []
    color_rgb = []
    
    num_points = 30
    x = np.linspace(-.35,1.5,num_points)
    y = np.linspace(0,1.3,num_points)
    z = np.linspace(-.25,1,num_points)
    counter = 0
    isFeasible = []
    for x_index in range(0,num_points):
        for y_index in range(0,num_points):
            for z_index in range(0,num_points):
                point.append([x[x_index],y[y_index],z[z_index]])
                plan, qf, plan_possible = generatePlan(q_initial, point[counter], gripperOri, tip_hand_transform, 'faster', plan_name = 'ik_test')
                if plan_possible:
                    plans.append(plan)
                    q_initial = qf
                    color_rgb.append((0,1,0,1))
                    isFeasible.append(True)
                else:
                    color_rgb.append((1,0,0,1))
                    isFeasible.append(False)
                counter+=1
    
    r = rospy.Rate(10)
    while True:
        plotBoxCorners(point,viz_pub,color_rgb = color_rgb, namespace = 'ik_points', isFeasible=isFeasible)
        r.sleep()
    ##################################
    
    #~ deleteMarkers(viz_pub,'object_bounding_box_for_collision')
    #~ deleteMarkers(viz_pub,'object_bounding_box_for_collision2')

    
