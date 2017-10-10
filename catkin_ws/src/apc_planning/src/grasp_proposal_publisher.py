#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 10 14:46:01 2017

@author: fhogan
"""

from planner_grasp import TaskPlanner
from placing_grasp import PlacingPlanner
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
    p.getPassiveVisionEstimate('update hm sg', '', p.tote_ID)
    number_bins = 2
    for bin_id in range(1,number_bins):
        print("getPassiveVisionEstimate 'update hm', '', ", bin_id)
        p.getPassiveVisionEstimate('update hm', '', bin_id)
    p.getPassiveVisionEstimate('update hm sg', '', 0)
    p.getBestGraspingPoint(0)
    

    ik.visualize_helper.visualize_grasping_proposals(p.proposal_viz_array_pub, np.asarray([p.grasp_point]),  p.listener, p.br, True)
    ik.visualize_helper.visualize_grasping_proposals(p.proposal_viz_array_pub, p.all_grasp_proposals,  p.listener, p.br)