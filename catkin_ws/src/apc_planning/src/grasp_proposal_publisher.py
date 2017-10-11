#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 10 14:46:01 2017

@author: fhogan
"""

from planner_grasp import TaskPlanner
from placing_grasp import PlacingPlanner
from visualization_msgs.msg import MarkerArray
from ik.marker_helper import  createDeleteAllMarker
import  optparse, rospy, sys
import ik
import numpy as np


if __name__=='__main__':
    rospy.init_node('grasp_visualizer')
    
    
    ## Parse arguments
    parser = optparse.OptionParser()
    
    (opt, args) = parser.parse_args()
    parser.add_option('-v', '--vision', action='store', dest='visionType',
        help='real or virtual', default='real')
    parser.add_option('--pvfile', action='store', dest='passive_vision_file_id',
        help='Path of the passive vision file to use', default='full_bin')
    parser.add_option('-p', '--pause', action='store_true', dest='withPause',
        help='To pause or not', default=False)
    parser.add_option('-n', '--noexe', action='store_false', dest='isExecute',
        help='To execute or not', default=True)
    parser.add_option('-e', '--experiment', action='store_true', dest='experiment',
        help='Whether to run passive vision experiments', default=False)
    (opt, args) = parser.parse_args()

    p = TaskPlanner(opt)

    #~initialize passive vistion in all bins
    number_bins = 2
    for bin_id in range(number_bins):
        print("getPassiveVisionEstimate 'update hm sg', '', ", bin_id)
        p.getPassiveVisionEstimate('update hm sg', '', bin_id)
    p.getBestGraspingPoint(1)
    

    markers_msg = MarkerArray()
    m0 = createDeleteAllMarker('pick_proposals')
    markers_msg.markers.append(m0)
    for i in range(0,100):
        p.proposal_viz_array_pub.publish(markers_msg)
        
    ik.visualize_helper.visualize_grasping_proposals(p.proposal_viz_array_pub, p.all_grasp_proposals,  p.listener, p.br)
    ik.visualize_helper.visualize_grasping_proposals(p.proposal_viz_array_pub, np.asarray([p.grasp_point]),  p.listener, p.br,  is_selected =True)
    
    p.all_grasp_proposals = None