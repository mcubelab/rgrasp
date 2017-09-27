#!/usr/bin/python

## Imports from planner
import subprocess
import scipy.stats
import random as rand
import time, datetime
#import object_calib_new
from tabulate import tabulate
import gripper
import suction
import json
import tf
from ik.ik import *
import numpy as np
import tf.transformations as tfm
from manual_fit.srv import *
import rospy
import passive_vision.srv
import sensor_msgs.msg
import goToHome
#import percept
#from suction_pick16 import suction_pick, is_suction_side_req
#from picking16 import pick
import picking16
from grasping17 import grasp
import suction_cup_ori
from copy import deepcopy
from std_srvs.srv import Empty
from numpy import linalg as la
from apc.helper import loadHeuristic, loadHeuristic_2016, sortOrder, displayOrder, displayOrder_placing_2016, bin_id2num
#from pushing_tote16 import pushing_tote
import optparse
import math
#import push16
import suction
from fake_inputbag import make_sure_path_exists
import suctionflip
import time 
import datetime
# put shared function into ik.helper module
from ik.roshelper import poseTransform, lookupTransform,lookupTransformList, coordinateFrameTransform,coordinateFrameTransformList, visualizeObjects, pose2list, pubFrame, ROS_Wait_For_Msg
# from ik.helper import get_obj_vol,rotmatZ, getBinMouth, get_obj_dim, matrix_from_xyzquat, pauseFunc, distance_wall_tote, find_object_pose_type_2016, find_object_pose_type_2016_short, getObjCOM, quat_from_matrix, get_bin_cnstr, in_or_out, moveGripper, getBinMouthAndFloor
import spatula
from explainer import Explainer

# Additional imports
from suction_down_simple import suction_down_simple
import numpy.linalg as la
import realsense_camera.srv
import active_vision.srv
from flush_grasping17 import flush_grasp

# Global variable to hold log file reference
log_file = 0

# Call passive vision, save log file paths, and return suction points and grasp proposals
def call_passive_vision(bin_id=0):
    print('[Vision] Explicitly waiting 10 seconds for passive vision to catch up.')
    time.sleep(10)
    print('[Vision] Calling passive vision system for bin #{}.'.format(bin_id))
    get_passive_vision_estimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
    passive_vision_state = get_passive_vision_estimate('',bin_id)
    print('[Vision] Recieved suction points and grasp proposals.')

    # Log passive vision files
    global log_file
    log_file.write('passive_vision_files {}\n'.format(passive_vision_state.file_paths[0]))

    # Reshape data from passive vision
    num_grasp_params = 12
    num_suction_params = 7
    num_flush_grasp_params = 6
    all_suction_points = np.asarray(passive_vision_state.suction_points)
    all_suction_points = all_suction_points.reshape(len(all_suction_points)/num_suction_params,num_suction_params);
    all_grasp_proposals = np.asarray(passive_vision_state.grasp_proposals)
    all_grasp_proposals = all_grasp_proposals.reshape(len(all_grasp_proposals)/num_grasp_params,num_grasp_params);
    all_flush_grasp_proposals = np.asarray(passive_vision_state.flush_grasp_proposals)
    all_flush_grasp_proposals = all_flush_grasp_proposals.reshape(len(all_flush_grasp_proposals)/num_flush_grasp_params,num_flush_grasp_params);

    return all_suction_points,all_grasp_proposals,all_flush_grasp_proposals

# Call active vision, save log file paths, and return object recognition prediction
def call_active_vision(bin_id=0):
    print('[Vision] Calling active vision system to recognize object in gripper.')
    getActiveVisionRecognition = rospy.ServiceProxy('/active_vision/recognize', active_vision.srv.recognition)
    active_vision_data = getActiveVisionRecognition(bin_id)
    highest_confidence = 0
    predicted_object_idx = 0;
    for i in range(0,len(active_vision_data.object_confidence)):
        if active_vision_data.object_confidence[i] > highest_confidence:
            highest_confidence = active_vision_data.object_confidence[i]
            predicted_object_idx = i
    predicted_object_name = active_vision_data.object_name[predicted_object_idx]
    print('[Vision] Active vision prediction: {}'.format(predicted_object_name))

    # Log saved image files and camera intrinsics files
    global log_file
    log_file.write('active_vision_files {}\n'.format(active_vision_data.file_paths[0]))
    log_file.write('active_vision_prediction {}\n'.format(predicted_object_name))

    return predicted_object_name

# Pick one unknown object from pick_source using action_primitive
def blind_pick(listener, br, src_bin_id=0,dst_bin_id=3,action_primitive='suction'):
    print('[Vision] Planning to pick one unknown object from bin #{} using {}.'.format(src_bin_id,action_primitive))

    global log_file
    log_file.write('pick_source {}\n'.format(src_bin_id))
    log_file.write('pick_destination {}\n'.format(dst_bin_id))
    log_file.write('action_primitive {}-{}\n'.format(action_primitive,'tote'))

    # Find suction points and grasp proposals with passive vision
    (all_suction_points,all_grasp_proposals,all_flush_grasp_proposals) = call_passive_vision(src_bin_id)

    pick_successful = False;
    if action_primitive == 'suction':

        if all_suction_points.shape[0] == 0:
            print('[Vision] No suction points found for bin #{}.'.format(src_bin_id))
            return pick_successful

        # NOTE: REMOVE THIS WHEN SUCTION DIRECTION IS CONTROLLABLE
        # Only keep suction points that are valid for suction down (with surface normals close to gravity)
        updown_suction_points_indices = []
        for i in range(0,all_suction_points.shape[0]):
            angle2gravity = np.rad2deg(np.arctan2(la.norm(np.cross([all_suction_points[i,3],all_suction_points[i,4],all_suction_points[i,5]],[0,0,1])),
                            np.dot([all_suction_points[i,3],all_suction_points[i,4],all_suction_points[i,5]],[0,0,1])))
            if angle2gravity < 20: # If angle to gravity direction is less than 20 degrees
                updown_suction_points_indices.append(i)
        updown_suction_points = np.zeros((len(updown_suction_points_indices),7))
        for i in range(0,len(updown_suction_points_indices)):
            updown_suction_points[i] = all_suction_points[updown_suction_points_indices[i],:]
        print(updown_suction_points.shape)
        all_suction_points = updown_suction_points

        suction_check = False
        num_tries = 0;
        while (not suction_check and num_tries < 3): # Keep trying until suction gets an object (max: 3 tries)

            # Randomly pick a valid suction point from top 50 and parse suction point output
            selected_suction_point = all_suction_points[num_tries,:]
            print('[Vision] Selected suction point: {}'.format(selected_suction_point))
            suction_target = [selected_suction_point[0],selected_suction_point[1],selected_suction_point[2]]
            surface_normal = [0,0,1]

            # Forcefully aim 2cm below suction point
            suction_target[2] = suction_target[2] - 0.05;

            # Call suction primitive
            # suction_output = suction_down_simple(input_speed='fastest',withPause=False,suction_position_target=suction_target,
            #                                      surface_normal=[0,0,1],tote_bounds=[.78,1.23,-.51,-.28,-.2,.1],
            #                                      flag=0,run_real=True,bin_id=src_bin_id,print_messages=False)

            suction_output = suction_down_simple(listener=listener, br=br, withPause=False,suction_position_target=suction_target,
                                                 surface_normal=[0,0,1],flag=0,bin_id=src_bin_id)
            suction_down_simple(listener=listener, br=br,withPause=False, flag=1)
            suction_check = suction_output.get('suction_check')
            log_file.write('selected_suction_point {}\n'.format(selected_suction_point))
            log_file.write('suction_success {}\n'.format(suction_check))
            num_tries += 1
        pick_successful = suction_check;

        # If successful suction... 
        if pick_successful:

            # Move object to base position above tote for active vision 
            suction_down_simple(listener=listener, br=br, input_speed='fastest',withPause=False,suction_position_target=suction_target,
                                surface_normal=[0,0,1],tote_bounds=[.78,1.23,-.51,-.28,-.2,.1],
                                flag=1,run_real=True,bin_id=src_bin_id,print_messages=False)

            # Perform recognition of object in gripper with external/active vision
            predicted_object_name = call_active_vision(src_bin_id)

            # Drop object over destination (random location, fixed z)
            # if dst_bin_id==0:
            #     tote_bounds=[.78,1.23,-.51,-.28,-.2,.1]
            # if dst_bin_id==1:
            #     tote_bounds=[.78,1.23,-.12,.12,-.2,.1]
            # if dst_bin_id==2:
            #     tote_bounds=[.78,1.23,.28,.51,-.2,.1]

            # drop_position = [np.random.uniform(tote_bounds[0],tote_bounds[1]),
            #                  np.random.uniform(tote_bounds[2],tote_bounds[3]),
            #                  0.1]
            # print(drop_position)
            suction_down_simple(listener=listener, br=br, withPause=False,
                                flag=2,run_real=True,bin_id=dst_bin_id,print_messages=False)

    if action_primitive == 'grasp':
        can_attempt_grasp = False
        pick_successful = False
        num_tries = 0
        max_tries = 10 # max attempts: 10 tries

        # If there are no grasp proposals, return pick_successful = False
        num_grasp_proposals = all_grasp_proposals.shape[0] + all_flush_grasp_proposals.shape[0]
        if num_grasp_proposals == 0:
            print('[Vision] No grasp proposals found for bin #{}.'.format(src_bin_id))
            log_file.write('\n')
            return pick_successful

        # Go through grasp proposals in order and check if it there is collision 
        grasp_proposal_idx = 0
        flush_grasp_proposal_idx = 0
        while (not can_attempt_grasp and num_tries < min(num_grasp_proposals,max_tries)):

            # Alternate between normal grasp and flush grasp in order of confidence
            if all_flush_grasp_proposals.shape[0] == 0 or (all_grasp_proposals.shape[0] > 0 and all_grasp_proposals[grasp_proposal_idx,11] > all_flush_grasp_proposals[flush_grasp_proposal_idx,5]):
                selected_grasp_type = 'normal'
                selected_grasp_proposal = all_grasp_proposals[grasp_proposal_idx,:];
                grasp_proposal_idx += 1
            else:
                selected_grasp_type = 'flush'
                selected_grasp_proposal = all_flush_grasp_proposals[flush_grasp_proposal_idx,:];
                flush_grasp_proposal_idx += 1
            print('[Vision] Selected grasp type: {}'.format(selected_grasp_type))
            print('[Vision] Selected grasp proposal: {}'.format(selected_grasp_proposal.tolist()))
            log_file.write('selected_grasp_type {}\n'.format(selected_grasp_type))
            log_file.write('selected_grasp_proposal {}\n'.format(selected_grasp_proposal.tolist()))

            # Check for collision and execute grasp if possible
            if selected_grasp_type == 'normal':
                grasp_output = grasp(objInput = selected_grasp_proposal,
                                     listener=listener,
                                     br=br,
                                     isExecute = True,
                                     objId = 'cheezit_big_original',
                                     binId = src_bin_id,
                                     flag = 0,
                                     withPause = True)
            else:
                grasp_output = flush_grasp(objInput = selected_grasp_proposal,
                                           listener=listener,
                                           br=br,
                                           isExecute = True,
                                           objId = 'cheezit_big_original',
                                           binId = src_bin_id,
                                           flag = 0,
                                           withPause = True)
            collision_detected = grasp_output.get('collision')
            execution_possible = grasp_output.get('execution_possible')
            can_attempt_grasp = not collision_detected
            object_in_grasp = execution_possible
            log_file.write('collision_detected {}\n'.format(collision_detected))
            log_file.write('object_in_grasp {}\n'.format(execution_possible))
            num_tries += 1

        pick_successful = can_attempt_grasp and object_in_grasp;

        # If successful grasp... 
        if pick_successful:

            # Perform recognition of object in gripper with external/active vision
            grasping_output_home = grasp(objInput = selected_grasp_proposal, listener=listener, br=br, isExecute=True, objId ='cheezit_big_original', binId= src_bin_id, flag = 1, withPause = True)
            time.sleep(3) # Wait for gripper to turn
            predicted_object_name = call_active_vision(src_bin_id)

            # Drop object in other tote
            # suction_down_simple(listener=listener, br=br, input_speed='fastest',withPause=True,suction_position_target=[1,0,1],
            #                     surface_normal=[0,0,1],tote_bounds=[.78,1.23,-.51,-.28,-.2,.1],
            #                     flag=1,run_real=True,bin_id=dst_bin_id,print_messages=False)
            grasp_output = grasp(objInput = selected_grasp_proposal,
                                 listener=listener,
                                 br=br,
                                 isExecute = True,
                                 objId = 'cheezit_big_original',
                                 binId = dst_bin_id,
                                 flag = 2,
                                 withPause = True)

    print('[Vision] Pick successful: {}'.format(pick_successful))
    log_file.write('pick_successful {}\n'.format(pick_successful))

    # Manually check which object was picked
    if pick_successful:
        manual_pick_successful = raw_input("Was it a successful pick? [y/n] ")
        while manual_pick_successful != 'n' and manual_pick_successful != 'y':
            manual_pick_successful = raw_input("Was it a successful pick? [y/n] ")
        manual_pick_successful = (manual_pick_successful == 'y')
        pick_successful = manual_pick_successful
        log_file.write('manual_pick_successful {}\n'.format(manual_pick_successful))
        if manual_pick_successful:
            manual_object_name = raw_input("Please type in the object name: ")
            log_file.write('manual_object_name {}\n'.format(manual_object_name))

        # Update state of bin #3 by sending command to passive vision
        if manual_pick_successful:
            if src_bin_id == 0 and dst_bin_id == 3: # Putting object into storage system (bin #3)
                get_passive_vision_estimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
                passive_vision_command_string = '{} add {}'.format(dst_bin_id,manual_object_name)
                passive_vision_command_bin_id = dst_bin_id
            elif src_bin_id == 3 and dst_bin_id == 0: # Taking object out of storage system (bin #3)
                get_passive_vision_estimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
                passive_vision_command_string = '{} rm {}'.format(src_bin_id,manual_object_name)
                passive_vision_command_bin_id = src_bin_id
            get_passive_vision_estimate(passive_vision_command_string,passive_vision_command_bin_id)
            log_file.write('passive_vision_command {}\n'.format(passive_vision_command_string))

    log_file.write('\n')
    return pick_successful

# ----------------------------------------------------------------------
# Instructions for looping (updated 5/23):
# 1. Run ROS RealSense cameras: rosrun realsense_camera stream _display:=true (run in pman, check depth is streaming properly in visualization)
# 2. Check that cameras 614203000465 and 612203002922 and 612203004574 and 616205005772 are connected
# 3. Make sure both totes are empty
# 4. rosrun passive_vision estimate _bin0_active:=true _bin1_active:=false _bin2_active:=false _bin3_active:=true (check that it is successfully running in pman or in console, if not, restart it and wait until keyword 'Ready.')
# 5. rosrun active_vision recognize (check that it is successfully running in pman, if not, restart it and wait until keyword 'Ready.')
# 6. Place objects in tote
# 7. See debug images in /home/mcube/arcdata/tmpdata/debug.png (should refresh itself at every iteration of passive vision)
# 8. Run this script: python looping_planner.py (change the main function to do suction or grasping or both)
# ----------------------------------------------------------------------
if __name__=='__main__':

    # ROS setup
    rospy.init_node('looping17', anonymous=True)

    # # Create directory for current looping session
    # data_directory = '/home/mcube/arcdata/loopdata/planner/{}'.format(int(round(time.time())))
    # os.makedirs(data_directory)
    # log_file = open(data_directory + '/{}.txt'.format(int(round(time.time()))), 'w')


    listener = tf.TransformListener()
    rospy.sleep(0.51)

    # parser = optparse.OptionParser()
    # parser.add_option('-l', '--loop', action='store_true', dest='loop_mode', help='Is looping between bin 0 and bin 3', default=False)
    # parser.add_option('-s', '--suction', action='store_true', dest='suction_only', help='Is testing suction only', default=False)
    # parser.add_option('-g', '--grasp', action='store_true', dest='grasping_only', help='Is testing grasping only', default=False)
    # (opt, args) = parser.parse_args()
    # loop_mode = opt.loop_mode
    # suction_only = opt.suction_only
    # grasping_only = opt.grasping_only

    # Move robot to home
    goToHome.goToARC(slowDown = True)

    print('Calling active vision system to recognize object in gripper.')
    getActiveVisionRecognition = rospy.ServiceProxy('/active_vision/recognize', active_vision.srv.recognition)
    active_vision_data = getActiveVisionRecognition(0, 0.0, ['scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges'], ['scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges','scotch_sponges'])
    prediction_full_list = active_vision_data.prediction_full_list;
    prediction_small_list = active_vision_data.prediction_small_list;
    predicted_object_name = prediction_full_list;

    print(predicted_object_name)
    print(prediction_small_list)
    print(not prediction_small_list) # prints true if prediction_small_list is empty
    
    # highest_confidence = 0
    # predicted_object_idx = 0
    # print('object_confidence_with_weights: ', active_vision_data.object_confidence_with_weights)
    # print('object_name: ', active_vision_data.object_name)
    # for i in range(0, len(active_vision_data.object_confidence_with_weights)):
    #     if active_vision_data.object_confidence_with_weights[i] > highest_confidence:
    #         highest_confidence = active_vision_data.object_confidence_with_weights[i]
    #         predicted_object_idx = i
    # predicted_object_name = active_vision_data.object_name[predicted_object_idx]
    


    # # Test suction only
    # if suction_only:
    #     while True:
    #         pick_successful = True
    #         while pick_successful: # Keep picking from bin #0 until failure
    #             pick_successful = blind_pick(listener=listener,src_bin_id=0,dst_bin_id=1,action_primitive='suction')
    #         pick_successful = loop_mode
    #         while pick_successful: # Keep picking from bin #1 until failure
    #             pick_successful = blind_pick(listener=listener,src_bin_id=1,dst_bin_id=0,action_primitive='suction')
        
    # # Test grasp only
    # if grasping_only:
    #     while True:
    #         pick_successful = True
    #         while pick_successful: # Keep picking from bin #0 until failure
    #             pick_successful = blind_pick(listener=listener,src_bin_id=0,dst_bin_id=3,action_primitive='grasp')
    #         pick_successful = loop_mode
    #         while pick_successful: # Keep picking from bin #3 until failure
    #             pick_successful = blind_pick(listener=listener,src_bin_id=3,dst_bin_id=0,action_primitive='grasp')

    # # Test both suction and grasping (toggle)
    # if not suction_only and not grasping_only:
    #     while True:
    #         pick_successful = True
    #         while pick_successful: # Keep picking from bin #0  with grasping until failure
    #             pick_successful = blind_pick(listener=listener,src_bin_id=0,dst_bin_id=3,action_primitive='grasp')
    #         pick_successful = True
    #         while pick_successful: # Keep picking from bin #0  with suction until failure
    #             pick_successful =blind_pick(listener=listener,src_bin_id=0,dst_bin_id=3,action_primitive='suction')
    #         pick_successful = loop_mode
    #         while pick_successful: # Keep picking from bin #3 until failure
    #             pick_successful = blind_pick(listener=listener,src_bin_id=3,dst_bin_id=0,action_primitive='grasp')
    #         pick_successful = loop_mode
    #         while pick_successful: # Keep picking from bin #3  with suction until failure
    #             pick_successful =blind_pick(listener=listener,src_bin_id=3,dst_bin_id=0,action_primitive='suction')













