#!/usr/bin/python

#rosrun realsense_camera stream _display:=true _save_data:=true
#rosrun passive_vision estimate _camera:="614203000682"
#rosservice call /passive_vision/estimate [] 0
#ssh -X rpi@192.168.0.249, before 171
# suction down primitive:
#rosservice call /suction_service "son" ""
# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
## The vertical dimension of the object should be smaller than the maximum
# gap distance between the two fingers.
from item import Item
import copy
from placing_planner17 import PlacingPlanner
import random, subprocess, time, datetime, json, math, optparse, rospy
import scipy.stats
import random as rand
import gripper
import tf
from ik.helper import get_params_yaml, vision_transform_precise_placing_with_visualization, fake_bbox_info_1
from ik.visualize_helper import visualize_flush_proposals, visualize_suction_points, visualize_suction_point, visualize_grasping_proposals
import numpy as np
import tf.transformations as tfm
from manual_fit.srv import *
import os
from collision_detection.collisionHelper import collisionCheck
import suction
from sensor_msgs.msg import Image as RosImage
#import shelf_helper
import webpages
#from collision_free_placing import collision_free_placing, go_arc_safe
import json

try:
    import passive_vision.srv
    import active_vision.srv
    import realsense_camera.srv
except:
    print 'FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL'

import sys
sys.path.append(os.environ['ARC_BASE']+'/catkin_ws/src/weight_sensor/src')
import ws_prob
import sensor_msgs.msg
import goToHome
from grasping17 import grasp
from flush_grasping17 import flush_grasp
from copy import deepcopy
from std_srvs.srv import Empty
from numpy import linalg as la
from fake_inputbag import make_sure_path_exists
from suction_down_simple import suction_down_simple
import pylab
import Image
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

from ik.roshelper import poseTransform, lookupTransform, lookupTransformList, coordinateFrameTransform, coordinateFrameTransformList, visualizeObjects, pose2list, pubFrame, ROS_Wait_For_Msg
from ik.helper import get_obj_vol, rotmatZ, get_obj_dim, matrix_from_xyzquat, pauseFunc, getObjCOM, quat_from_matrix, in_or_out, moveGripper, fake_bbox_info
from ik.marker_helper import createArrowMarker, createCubeMarker2, createDeleteAllMarker
from visualization_msgs.msg import MarkerArray, Marker
import spatula
from explainer import Explainer
from ik.helper import Timer

import cv2
import Tkinter as tk # sudo apt-get install python-tk
import tkMessageBox
from placingInterface import PlacingInterface
from apc.helper import UpdateCommand


class TaskPlanner(object):
    def __init__(self, args):
        #######################
        ### Class constants ###
        #######################
        self.passiveVisionTypes = {'real' : self.call_passive_vision,
                                   'file' : self.call_passive_vision,
                                   'virtual' : self.callFakePassiveVision}
        self.deciderTypes = {'experiment' : None,
                             'baseline' : None,
                             'random' : None}
        self.all_primitives = ['suction-tote', 'grasp-tote']
        self.picking_preparation = {'baseline' : self.pickingPreparation}
        self.preparation = {'experiment' : self.experimentPreparation,
                            'baseline' : self.baselinePreparation,
                            'random': self.baselinePreparation}
        self.draw = {'experiment' : self.experiment_draw,
                     'baseline' : self.baseline_draw,
                     'random': self.random_draw}
        self.pick = {'baseline' : self.baseline_pick}
        self.placer = {'experiment' : self.planned_place,
                       'baseline' : self.planned_place,
                       'random': self.planned_place}
        self.store_info = {'experiment' : self.experiment_logging,
                           'baseline' : self.baseline_logging,
                           'random': self.baseline_logging}
        self.FAKE_PASSIVE_VISION_DIR = os.environ['ARC_BASE'] + '/input/fake_dirs/fake_passive_vision/'
        self.FAKE_GRASPING_DIR = os.environ['ARC_BASE'] + '/input/fake_dirs/fake_grasping/'
        self.FAKE_SUCTION_DIR = os.environ['ARC_BASE'] + '/input/fake_dirs/fake_suction/'
        self.param_suction = 7
        self.param_suction_side = 7
        self.param_grasping = 12
        self.param_flush = 6
        self.penalty_bad_suction = 1.
        self.penalty_bad_grasping = 1.
        self.passive_vision_file_path = 'full_bin'
        self.num_attempts = 0
        self.num_attempts_failed = 0
        ##################################
        ### Assertions before starting ###
        ##################################
        assert ((not opt.forceSucc) or opt.in_simulation), 'Can only force success in simulation'
        assert opt.visionType in self.passiveVisionTypes, 'visionType should be in'+str([_ for _ in self.passiveVisionTypes])+'but its '+opt.visionType
        assert (opt.visionType == 'real' or (not opt.experiment)), 'visionType should be real during experiments'
        assert (opt.in_simulation or opt.visionType != 'virtual'), 'we should be in simulation to use virtual vision'
        e.db(visionType = opt.visionType)
        e.db(in_simulation = opt.in_simulation)
        e.db(combined_task = opt.combined_task)
        assert (opt.visionType == 'virtual' or (not opt.in_simulation)), 'visionType should be virtual in simulation'
        assert (not opt.experiment or not opt.in_simulation), 'Cannot do experiments in simulation'
        assert (opt.decider in self.deciderTypes), 'Decider should be in '+str([_ for _ in self.deciderTypes])
        assert (opt.json_order_filename == '' or opt.task=='picking'), 'You can only have json_order_filename in picking'
        #Check path for jsonfile
        self.jsonfilename = opt.jsonfilename
        self.json_order_filename = opt.json_order_filename
        self.interactive_filename = opt.interactive_filename
        self.json_boxes_filename = 'box_sizes.json'
        if not os.path.isfile(self.jsonfilename):
            self.jsonfilename = os.environ['ARC_BASE'] + '/input/' + self.jsonfilename
        if not os.path.isfile(self.json_order_filename):
            self.json_order_filename = os.environ['ARC_BASE'] + '/input/' + self.json_order_filename
        if not os.path.isfile(self.interactive_filename):
            self.interactive_filename = os.environ['ARC_BASE'] + '/input/' + self.interactive_filename
        if not os.path.isfile(self.json_boxes_filename):
            self.json_boxes_filename = os.environ['ARC_BASE'] + '/input/' + self.json_boxes_filename
        #assert os.path.isfile(self.jsonfilename), 'JSON file leads nowhere'
        #####################
        ### Configuration ###
        #####################
        self.in_simulation = opt.in_simulation
        self.withPause = opt.withPause
        self.forceSucc = opt.forceSucc
        self.duration_of_contest = int(opt.duration_of_contest)
        self.num_pick = int(opt.num_pick)
        self.num_combined_stow = int(opt.num_combined_stow)
        self.experiment = opt.experiment
        self.task = opt.task
        self.isExecute = opt.isExecute
        self.human_supervision = opt.human_supervision
        self.combined_task = opt.combined_task
        self.without_active_vision = opt.without_active_vision
        self.container_where_robot_did_palcing = None

        #Must the object go into a box? In stowing no object will have to, so by default the answer is no.
        #This is the default value for the task
        self.ini_is_goal = 'no' if self.task=='stowing' else 'maybe'
        self.passive_vision_file_id = opt.passive_vision_file_id
        #######################
        ### Class variables ###
        #######################
        self.pickedObjects = []
        self.tote_ID = 0
        self.num_bins = 3
        self.num_containers = self.num_bins+1
        self.num_boxes = 5
        self.box_id = ['1A5', '1AD', 'A1', '1B2', 'K3']
        self.box_sizes = [[] for _ in range(self.num_boxes)]
        self.goals = []
        self.goals_left = 0 #number of objects to be placed in boxes
        self.objects = [] #List of objects in this run [in all places]
        self.toteObj = [] #List of indices to the object list of objects in tote
        self.shelfObj = [] #List of indices to the object list of objects in shelf
        self.initial_objects_bins = [[] for _ in range(self.num_containers)]
        self.initial_num_goals = 10
        self.suction_side = False
        self.previous_primitive = None
        self.times_no_points_in_container = [0 for _ in range(self.num_containers)]
        self.failed_attempts_in_high_surface = 0
        self.bin1_pose = get_params_yaml('bin1_pose')
        if self.experiment:
            self.init_experiment(opt) #initialize things for experiments
        else:
            #self.duration_of_contest = 15*60 #15 min
            self.decider = opt.decider
        #Vision type
        self.visionType = opt.visionType

        # For grasping test
        self.only_flush = False
        if opt.primitives == 'flush-tote':
            self.only_flush = True
            opt.primitives = 'grasp-tote'
        self.only_grasp = False
        if opt.primitives == 'only-grasp-tote':
            self.only_grasp = True
            opt.primitives = 'grasp-tote'

        #Set primitives
        self.primitives = []
        for primitive in self.all_primitives:
            if (primitive in opt.primitives) or (opt.primitives == 'all'):
                self.primitives.append(primitive)
        self.fails_in_row = 0
        self.with_suction_side = True
        ## Manipulation
        self.goHomeSlow = False
        self.primitive_version = 1
        self.dirty_bins = ["0"]*4 #All bins start clean
        self.placedObj = None
        self.time_since_clean = [math.floor(time.time()-10)]*4
        self.primitive_id = 'Null'
        self.suction_side_score = 0
        self.suction_side = False
        ## ROS setup
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.number_times_wrong_weight = 0
        #self.publisher = rospy.Publisher('dirty_bins', std_msgs.msg.String, queue_size=10)
        self.go_faster_flag = False
        if self.visionType == 'real':
            self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
        '''
        #JSON
        self.use_last_json = False
        if opt.interactive_filename is '':
            self.enter_jsonfile()
        '''
        self.list_objects_in_bin_1 = ["composition_book", "tennis_ball_container", "purple_binder", "toilet_brush", "hanes_socks", "gray_divided_mesh_cup"]
        #["ratchet", "hand_weight", "rudolph_erasers", "reynolds_wrap", "robots_everywhere", "composition_book", "soap_dish", "condiment_bottles", "balloons", "tennis_ball_container", "scotch_sponges", "white_facecloth", "r2d2_holo_notebook", "purple_binder", "toilet_brush", "hanes_socks", "shoe_polish", "gray_divided_mesh_cup", "claw_hair_clips", "irish_spring_soap", "bubble_blower", "mermaid_crayons", "speed_stick", "flashlight", "table_cloth", "steel_scourer", "black_fashion_gloves", "heat_wrap", "measuring_spoons", "brillo_pads", "gardeners_select_gloves", "light_bulb"]
        #["bath_sponge", "reynolds_wrap", "composition_book", "epsom_salts", "tennis_ball_container", "folgers_classic_roast_coffee", "toilet_brush", "hanes_socks", "ice_cube_tray", "mesh_cup", "windex"] 

        #Bad points
        self.bad_suction_points = [] #list of failed attempts
        self.bad_suction_side_points = []
        self.bad_grasping_points = []
        self.bad_suction_times = [] #list of times when we failed
        self.bad_suction_side_times = []
        self.bad_grasping_times = []

        #Placing
        self.PlacingPlanner = PlacingPlanner(e = e, visionType = self.visionType)
        self.PlacingPlanner.put_objects_from_planner(self.objects)
        self.PlacingPlanner.task = self.task
        self.PlacingPlanner.combined_task = self.combined_task
        self.PlacingPlanner.time_since_clean = self.time_since_clean
        #WeightF
        self.weightSensor = ws_prob.WeightSensor()
        self.withSensorWeight = True
        if self.in_simulation == True:
            self.withSensorWeight = False

        # Initialize webpage
        self.webpage = webpages.WebDisplay()

        ########################################
        ### Potentially not useful variables ###
        ########################################
        self.weight_info = [None for _ in range(9)]  #Adding the boxes
        self.weight_info_after_place = [None for _ in range(9)] #Adding the boxes
        self.vision_info = None
        self.active_vision_ID = None
        self.active_vision_ID_small_list = None
        e.db(decider=self.decider)
        e.db(experiment=self.experiment)

        ########################################
        ###         Class Publishers         ###
        ########################################
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.proposal_viz_array_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
        self.hm_pre_pub = rospy.Publisher('/height_map_pre', RosImage, queue_size = 10)
        self.hm_post_pub = rospy.Publisher('/height_map_post', RosImage, queue_size = 10)
        self.score_pub = rospy.Publisher('/placing_score', RosImage, queue_size = 10)
        rospy.sleep(0.5)


    ###############################
    ### GLOBAL HELPER FUNCTIONS ###
    ###############################

    def clean_bin(self, bin_num = -1):
        #bin_num = -1 updates all of the bins
        update_dirty_bins_msg = 'dirty '
        for i in range(4):
            if bin_num == -1 or bin_num == i:
                self.dirty_bins[i]='0'
                self.time_since_clean[i] = math.floor(time.time())
                e.db('Updated clean state of ', i, 'with time: ', self.time_since_clean[i])
            update_dirty_bins_msg += self.dirty_bins[i]


    def dirty_bin(self, bin_num = -1):
        #bin_num = -1 updates all of the bins
        update_dirty_bins_msg = 'dirty '
        for i in range(4):
            if bin_num == -1 or bin_num == i:
                self.dirty_bins[i]='1'
            update_dirty_bins_msg += self.dirty_bins[i]
        #if self.visionType == 'real':
        #    self.getPassiveVisionEstimate(update_dirty_bins_msg, 0)

    def request_passive_vision_wait(self, bin_id):
        while True:
            try:
                return self.getPassiveVisionEstimate('request', '', bin_id)
            except:
                e.db('Waiting Vision.')

    def enter_jsonfile(self):
        '''Extracts the info from the input jsonfile'''
        if self.experiment and (not self.use_last_json):
            self.talkWithHumanToStartExperiment()
        else:
            with open(self.jsonfilename) as data_file:
                DATA = json.load(data_file)

            ##Introduce objects in tote
            for obj in DATA['tote']['contents']:
                self.objects.append(Item(container=0,is_goal=self.ini_is_goal))
                self.objects[-1].update_with_label(obj)
                if self.task == 'stowing':
                    self.initial_objects_bins[self.tote_ID].append(obj)            
            

            should_introduce_shelf = True
            for x in self.objects:
                if x.container in range(1,4): # detected object in bin
                    should_introduce_shelf = False
            if should_introduce_shelf: #self.in_simulation or (opt.interactive_filename is not '') or (self.combined_task and self.task == 'picking') or (not self.combined_task and self.task == 'stowing'):
                ##Introduce objects in shelfs
                should_fill = True
                for x in self.initial_objects_bins:
                    if len(x) != 0:
                        should_fill = False
                for x in range(len(DATA['bins'])):
                    for obj in DATA['bins'][x]['contents']:
                        bin_num = x+1
                        if x >= 3: #More bins given than current system has
                            bin_num = 1 #Put in big bin
                        self.objects.append(Item(container=bin_num,is_goal=self.ini_is_goal))
                        self.objects[-1].update_with_label(obj)
                        if should_fill:#self.task == 'stowing' or self.combined_task or opt.interactive_filename is not '':
                            self.initial_objects_bins[bin_num].append(obj)
                        #TODO_M: these needs to come from the previous placing of the picking objects
            self.toteObj = []
            self.shelfObj = []
            for i,obj in enumerate(self.objects):
                if obj.container == self.tote_ID:
                    self.toteObj.append(i)
                else:
                    self.shelfObj.append(i)
                    
            '''
            self.toteObj = range(initial_objects,len(self.objects))
            if self.combined_task and self.task == 'stowing':
                self.shelfObj = range(self.toteObj[-1]+1, len(self.objects))
            else:
                self.shelfObj = range(len(DATA['tote']['contents']), len(self.objects))
                
            '''
        '''
            if self.use_last_json:
                e.db(toteObjNames=self.toteObjNames)
                finished_placing = 'n'
                while finished_placing != 'y':
                    finished_placing = raw_input('Have you finished placing the objects?[y/n]')

        if not self.use_last_json and self.task=='stowing': #TODO: change to allow input from picking
            DATA = {}
            DATA['tote'] = {}
            DATA['tote']['contents'] = self.toteObjNames()
            DATA['bins'] = []
            for x in range(self.num_bins):
                DATA['bins'].append({'bin_id': chr(ord('A')+x), 'contents': self.shelfObjNamesGivenBin(x+1) })
            with open(os.path.join(os.environ['ARC_BASE'], 'input/last_input.json'), 'w') as outfile:
                json.dump(DATA, outfile, sort_keys=True, indent=4,
                          ensure_ascii=False)
        '''

    def enter_jsonfile_boxes(self):
            ##Introduce boxes
            if self.json_boxes_filename != os.environ['ARC_BASE'] + '/input/':  #No file name assigned to it
                with open(self.json_boxes_filename) as data_file:
                    DATA_JSON = json.load(data_file)
                for box_num, box in enumerate(DATA_JSON['boxes']):
                    box_index = self.box_id.index(box['size_id'])
                    self.box_sizes[box_index] = box['dimensions']

    def enter_jsonfile_goals(self):
            ##Introduce goal objects
            if self.json_order_filename != os.environ['ARC_BASE'] + '/input/':  #No file name assigned to it
                with open(self.json_order_filename) as data_file:
                    DATA_JSON = json.load(data_file)
                self.goals_left = 0
                self.box_goals = [[] for _ in range(self.num_boxes)] #labels of desired object per box
                for box_goal in DATA_JSON['orders']:
                    for box_num, box_id in enumerate(self.box_id):
                        if box_id == box_goal['size_id']:
                            self.goals_left += len(box_goal['contents']) #increase number of goals
                            #Update every object in the shelf as potentially a goal
                            self.box_goals[box_num] = copy.deepcopy(box_goal['contents'])
                            for goal_label in box_goal['contents']:
                                the_goal_is_there = False
                                for obj in self.objects:
                                    if obj.label == goal_label:
                                        obj.is_goal = 'yes'
                                        self.goals.append(obj.label)
                                        the_goal_is_there = True
                                if not the_goal_is_there:
                                    self.goals_left -= 1

                            # TODO fix repeated and goals that ar
                self.initial_num_goals = self.goals_left
                self.webpage.updateValue('AllGoals', self.goals)
                self.webpage.updateValue('NewGoals', self.goals)
            if self.task == 'picking':
                #In picking, we know whether you're a goal or not; there's no maybe
                for obj in self.objects:
                    #Convert 'maybe' in 'yes' or 'no'
                    if obj.is_goal != 'yes':
                        obj.is_goal = 'no'

    def numGoalObjectsContainer(self, container):
        num = 0
        for obj in self.objects:
            if obj.container == container and obj.is_goal == 'yes':
                num += 1
        return num

    def printObjects(self, cont_filter = None, withPause = True):
        '''Prints details of the objects that are in the containers cont_filter'''
        e.db('-'*25)
        e.db('OBJECTS IN ', cont_filter if cont_filter != None else 'ALL CONTAINERS')
        for obj in self.objects:
            if cont_filter == None or obj.container in cont_filter:
                e.db(obj)
        e.db('-'*25)

    def toteObjNames(self):
        '''Returns the list of names of objects in tote'''
        return [self.objects[_].label for _ in self.toteObj]

    def shelfObjNames(self):
        '''Returns the list of names of objects in the shelf'''
        return [self.objects[_].label for _ in self.shelfObj]

    def shelfObjNamesGivenBin(self, bin_num):
        '''Returns the list of names of objects in a particular bin of the shelf'''
        objects_in_bin = []
        for obj in self.shelfObj:
            if self.objects[obj].container == bin_num:
                objects_in_bin.append(self.objects[obj].label)
        return objects_in_bin

    def shelfObjNamesGivenBox(self, bin_num):
        '''Returns the list of names of objects in a particular bin of the shelf'''
        objects_in_box = []
        for obj in self.objects:
            if obj.container == bin_num:
                objects_in_box.append(obj.label)
        return objects_in_box

    def containerObjNames(self, container):
        res = []
        for obj in self.objects:
            if obj.container == container:
                res.append(obj.label)
        return res

    def placedObject(self):
        return self.objects[self.placedObj]

    def getWeightProbabilities(self):
        '''Get the weighted probabilities of item in the tote'''
        self.weightProb = [1./len(self.toteObj) for _ in range(len(self.toteObj))]

    def getSegmentationProbabilities(self):
        self.segProb = [1./len(self.toteObj) for _ in range(len(self.toteObj))]

    def getProbabilities(self):
        self.getWeightProbabilities()
        self.getSegmentationProbabilities()
        self.finalProb = [(self.weightProb[i] + self.segProb[i])/2. for i in range(len(self.toteObj))]

    def adjustPredictionsWithFailedAttempts(self): #Not used
        curr_time = time.time()
        #Suction
        for i in range(len(self.all_suction_points)):
            prod_prob = 0
            for p,t in zip(self.bad_suction_points, self.bad_suction_times):
                weight_time = max(0., 1- (curr_time - t)/(2.*60)) #Different attempt after 2 min
                weight_dist = max(0., 1- (np.linalg.norm(np.asarray(p)-np.asarray(self.all_suction_points[i,:-1]))/0.15)) #Different proposal @15cm
                prod_prob -= self.penalty_bad_suction*weight_time*weight_dist
            prod_prob = 1./(1+math.exp(-prod_prob)) #sigmoid
            self.all_suction_points[i,-1] *= prod_prob
        #Grasp
        for i in range(len(self.all_grasp_proposals)):
            prod_prob = 0
            for p,t in zip(self.bad_grasping_points, self.bad_grasping_times):
                if len(p) != 11:
                    continue
                weight_time = max(0., 1- (curr_time - t)/(2.*60))
                weight_dist = max(0., 1- (np.linalg.norm(np.asarray(p[:-1])-np.asarray(self.all_grasp_proposals[i,:-1]))/0.15))
                prod_prob -= self.penalty_bad_grasping*weight_time*weight_dist
            prod_prob = 1./(1+math.exp(-prod_prob)) #sigmoid
            self.all_grasp_proposals[i,-1] *= prod_prob
        #Flush
        for i in range(len(self.all_flush_proposals)):
            prod_prob = 0
            for p,t in zip(self.bad_suction_points, self.bad_grasping_times):
                if len(p) != 5:
                    continue
                weight_time = max(0., 1- (curr_time - t)/(2.*60))
                weight_dist = max(0., 1- (np.linalg.norm(np.asarray(p)-np.asarray(self.all_flush_proposals[i,:-1]))/0.15))
                prod_prob -= self.penalty_bad_grasping*weight_time*weight_dist
            prod_prob = 1./(1+math.exp(-prod_prob)) #sigmoid
            self.all_flush_proposals[i,-1] *= prod_prob

    ##############################
    ### FAKE VIRTUAL FUNCTIONS ###
    ##############################
    def callFakeActiveVision(self, container = None):
        '''For fake active vision, pick a random label from the container'''
        if container == None:
            container = self.tote_ID
        self.pickedLabel = self.containerObjNames(container)[np.random.randint(len(self.containerObjNames(container)))]
        #bbox_info = [1,0,0,0,0.9949660234451294, -0.5, .2,0,0,0,0]
        bbox_info=fake_bbox_info(listener=self.listener)
        for i, obj in enumerate(self.objects):
            if obj.label == self.pickedLabel and obj.container == container:
                bbox_info[7:10] = obj.dim
        return self.pickedLabel, self.pickedLabel, bbox_info, bbox_info

    def callFakePassiveVision(self, container):
        '''For Fake Passive vision, read fake results of vision from a
        data file and pull out the suction and grasp proposals. Param container is unused'''
        e.db('File used for fake passive vision: ', self.passive_vision_file_id+ '/passive_vision_data_bin_' + str(container) + '.json')
        full_path = self.FAKE_PASSIVE_VISION_DIR + self.passive_vision_file_id
        with open(full_path + '/passive_vision_data_bin_' + '0' + '.json', 'r') as infile:
            DATA = json.load(infile)
        self.all_suction_points = [float(i) for i in DATA['suction_points']]
        self.all_suction_points = np.array(self.all_suction_points)
        self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/7, 7)
        self.all_suction_points = self.all_suction_points[self.all_suction_points[:, 7-1].argsort()[::-1]]
        self.all_suction_side_points = copy.deepcopy(self.all_suction_points)
        self.all_grasp_proposals = [float(i) for i in DATA['grasp_proposals']]
        self.all_grasp_proposals = np.array(self.all_grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/12, 12)

        # Check if flush proposals are in data. If so use them, otherwise populate as empy
        if 'flush_grasp_proposals' in DATA:
            self.all_flush_proposals = [float(i) for i in DATA['flush_grasp_proposals']]
            self.all_flush_proposals = np.array(self.all_flush_proposals)
            self.all_flush_proposals = self.all_flush_proposals.reshape(len(self.all_flush_proposals)/self.param_flush, self.param_flush)
        else:
            self.all_flush_proposals = np.array([])

    def callFakeGrasping(self, prob=-1., container = None):
        '''Call a fake grasping with a certain probabilitly of success'''
        if container == None:
            container = self.tote_ID
        self.dirty_bin(container)
        goToHome.prepGripperPicking()
        #Run the primitive in virtual
        if len(self.grasp_point) == self.param_flush or self.only_flush:
            e.db('------- DOING FLUSH ------- ')
            e.db( grasp_point = self.grasp_point)
            _ = flush_grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                            binId=container, flag=0, withPause=self.withPause, viz_pub=self.viz_array_pub)
        else:
            e.db('------- DOING GRASPING ------- ')
            e.db( grasp_point = self.grasp_point)
            _ = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                      binId=container, flag=0, withPause=False, viz_pub=self.viz_array_pub)
        ##Go to active vision position
        #pick(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute, binId=container, flag=1, withPause=self.withPause, viz_pub=self.viz_array_pub)
        #Get fake output for the primitive
        f = random.choice(os.listdir(self.FAKE_GRASPING_DIR))
        with open(os.path.join(self.FAKE_GRASPING_DIR, f), 'r') as infile:
            self.grasping_output = json.load(infile)
        self.grasping_output = self.grasping_output['primitive_output']

        #Decide if primitive succeeds or not given probability and flags
        if self.forceSucc:
            self.grasping_output['pick_possible'] = True
        elif prob >= 0.:
            self.grasping_output['pick_possible'] = (np.random.rand() <= prob)
        self.grasping_output['execution_possible'] = self.grasping_output['pick_possible']
        self.execution_possible = self.grasping_output['execution_possible']
        self.pickedLabel = 'unknown'
        self.clean_bin(container)

    def callFakeSuction(self, prob=-1., container = None):
        '''Call a fake grasping with a certain probabilitly of success'''
        if container == None:
            container = self.tote_ID
        self.dirty_bin(container)
        goToHome.prepGripperSuction()
        #Just run it for visualization purposes:
        suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause,
                        suction_position_target_list=[self.suction_point[0:3]],
                        flag=0, bin_id=container, viz_pub=self.viz_array_pub)
        #Get fake output for the primitive
        f = random.choice(os.listdir(self.FAKE_SUCTION_DIR))
        with open(os.path.join(self.FAKE_SUCTION_DIR, f), 'r') as infile:
            self.suction_output = json.load(infile)
        self.suction_output = self.suction_output['primitive_output']
        #Decide if primitive succeeds or not given probability and flags
        if self.forceSucc:
            self.suction_output['success_flag'] = True
        elif prob >= 0.:
            self.suction_output['success_flag'] = (np.random.rand() <= prob)
        self.suction_output['execution_possible'] = self.suction_output['success_flag']
        self.execution_possible = self.suction_output['execution_possible']
        self.pickedLabel = 'unknown'
        self.clean_bin(container)

    ################
    ### PICKING ####
    ################
    def choose_box_for_object(self, obj):
        '''Selects a box for the objects'''
        for box in range(self.num_boxes):
            for o in self.box_goals[box]:  #TODO_M: if there are multiple goals repeated this won't work!
                if o == obj.label:
                    obj.container = box + 4 #Because there are 4 containers
                    return
        e.db(obj.label, 'is not a goal object, so it is placed in a bin')

    def update_goals_after_pick(self, obj):
        '''Receives an object and updates information w.r.t. goals'''
        #Go through the goals for the box where we put the object
        for i,x in enumerate(self.box_goals[obj.get_box()]):
            if x == obj.label: #if label matches, remove it
                del self.box_goals[obj.get_box()][i]
                self.goals_left -= 1
                for i,goal in enumerate(self.goals):
                    if goal == obj.label:
                        self.goals.pop(i)
                        if obj.label not in self.goals: #All the objects with same ID now are no more goals
                            for obj2 in self.objects:
                                if obj2.label == obj.label:
                                    obj2.is_goal = 'no'
                self.webpage.updateValue('NewGoals', self.goals)
                return #we only want to remove one goal
        #TODO_M: update is_goal params in case same label no longer needed

    def baseline_pick(self):
        '''This function is in charge of deciding what to pick next'''
        #Get IDs of the containers [most likely 1,2,3]
        self.container_IDs = []
        e.db(num_attempts_failed = self.num_attempts_failed)
        e.db(number_goals_bin_1 = self.numGoalObjectsContainer(1))
        self.container_where_robot_did_placing = 1
        if self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0:
            for i in range(self.num_containers):
                if i != self.tote_ID:
                    self.container_IDs.append(i)
            # TODO: do we still want to do this? vision say yes
            if self.container_tried_to_pick is not None:
                index_container = self.container_IDs.index(self.container_tried_to_pick)
                self.container_IDs.pop(index_container)
                self.container_IDs.append(self.container_tried_to_pick)
            if self.container_where_robot_did_placing is not None:
                index_container = self.container_IDs.index(self.container_where_robot_did_placing)
                self.container_IDs.pop(index_container)
                self.container_IDs.append(self.container_where_robot_did_placing)
        else:
            self.container_IDs.append(1)  #only bin 1
        #Get the best suction and grasping points for every container
        self.best_points = []
        self.suction_side = False
        e.db(times_no_points_in_container = self.times_no_points_in_container)
        for container in self.container_IDs:
            if self.numGoalObjectsContainer(container) == 0 or self.times_no_points_in_container[container] > 10:
                continue
            if len(self.best_points) > 0:
                if self.container_tried_to_pick is not None and container == self.container_tried_to_pick:
                    continue
                if container == self.container_where_robot_did_placing:
                    continue
            if 'suction-tote' in self.primitives:
                self.getBestSuctionPoint(container)
                if len(self.bad_suction_points) > 1:
                    self.suction_score *= 0.5
                    if len(self.bad_suction_points) > 3:
                        self.suction_score *= 0.5
                self.best_points.append({'score' : self.suction_score,
                                                 'point': self.suction_point,
                                                 'container': container,
                                                 'primitive': 'suction-tote',
                                                 'suction-side': False})
                e.db('The suction_score considered is: ', self.suction_score)
                self.webpage.updateValue('SuctionScore', self.suction_score)
            if 'suction-tote' in self.primitives and self.with_suction_side and container == 1:
                self.getBestSuctionSidePoint(container)
                if len(self.bad_suction_side_points) > 1:
                    self.suction_side_score *= 0.5
                    if len(self.bad_suction_side_points) > 3:
                        self.suction_side_score *= 0.5
                self.suction_side_score *= 0.95  #Make it worse than suction
                self.best_points.append({'score' : self.suction_side_score,
                                                 'point': self.suction_side_point,
                                                 'container': container,
                                                 'primitive': 'suction-tote',
                                                 'suction-side': True})
                self.webpage.updateValue('SuctionSideScore', self.suction_side_score)
            if 'grasp-tote' in self.primitives:
                self.getBestGraspingPoint(container)
                if len(self.bad_grasping_points) > 1:
                    self.grasp_score *= 0.5
                    if len(self.bad_grasping_points) > 3:
                        self.grasp_score *= 0.5

                self.grasp_score *= 0.95
                e.db('The grasp_score considered is: ', self.grasp_score)
                self.best_points.append({'score' : self.grasp_score,
                                                 'point': self.grasp_point,
                                                 'container': container,
                                                 'primitive': 'grasp-tote',
                                                 'suction-side': False})
                self.webpage.updateValue('GraspingScore', self.grasp_score)


            self.all_suction_points = None #Needed to allow updates
        if len(self.best_points) == 0:
            e.db('No best point has been obtained, try to call baseline_pick again')
            self.pick[self.decider]()
            return

        #Sort the best points according to their score and select the best one
        self.best_points.sort(key = lambda x : x['score'], reverse = True)
        print self.best_points

        self.primitive_point = self.best_points[0]['point']
        self.primitive_score = self.best_points[0]['score']
        self.best_container = self.best_points[0]['container']
        self.best_primitive = self.best_points[0]['primitive']
        self.suction_side = self.best_points[0]['suction-side']
        self.webpage.updateValue('RobotAction', self.best_primitive)
        #Execute the best primitive
        if self.best_primitive == 'suction-tote':
            self.best_primitive = 'suction-tote'
            e.db('-----------------------------')
            e.db('RUNNING SUCTION IN CONTAINER ', self.best_container)
            e.db('-----------------------------')
            self.suction_point = copy.deepcopy(self.primitive_point)
            self.suction_score = copy.deepcopy(self.primitive_score)
            self.webpage.updateValue('SuctionScore', self.suction_score)
            self.primitive_id = copy.deepcopy(self.suction_id)
            self.primitive_id_confidence = copy.deepcopy(self.suction_id_confidence)
            if self.suction_side:
                    self.primitive_id = copy.deepcopy(self.suction_side_id)
                    self.primitive_id_confidence = copy.deepcopy(self.suction_side_id_confidence)
            e.db( 'The suction point is: ', self.suction_point, ' and has a score of: ', self.suction_score, '. Is suction side? ', self.suction_side)
            self.run_suction(container=self.best_container)
        else:
            self.best_primitive == 'grasp-tote'
            e.db('-----------------------------')
            e.db('RUNNING GRASPING IN CONTAINER', self.best_container)
            e.db('-----------------------------')
            self.grasp_point = copy.deepcopy(self.primitive_point)
            self.grasp_score = copy.deepcopy(self.primitive_score)
            self.primitive_id = copy.deepcopy(self.grasp_id)
            self.primitive_id_confidence = copy.deepcopy(self.grasp_id_confidence)
            e.db( 'The grasping point is: ', self.grasp_point, ' and has a score of: ', self.grasp_score)
            self.webpage.updateValue('GraspingScore', self.grasp_score)
            self.run_grasping(container=self.best_container)
        self.previous_primitive = copy.deepcopy(self.best_primitive)
        self.webpage.updateValue('RobotAction', self.best_primitive)
        self.webpage.updateValue('PrimitiveID', self.primitive_id)

####Not used####
    def getScorePickGoalObject(self, container, pos):
        # Receives a position and computes probability of picking correct object
        weights = {'vision' : 0, 'position' : 1.0, 'weight' : 0}
        score = 0.
        #Compute position score
        weight_goal = 0.
        weight_tot = 0.
        for o in self.objects:
            if self.container == container:
                w_o = 1.
            else:
                w_o = 0.
            weight_tot += w_o
            if o.label in self.goals:
                weight_goal += w_o
        score_pos = weight_goal/weight_tot
        score += weights['position']*score_pos
        return score
###############

    def pickingProposalsFilter(self, container):

        general_score = 1
        bad_visibility_score = 0.5
        if self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0:
            no_goal_score = 0.1
            if container == 1: #if not bin 1
                general_score = 0.001
            if self.num_attempts_failed > 20:
                no_goal_score = 0.8
                bad_visibility_score = 1
        else:
            no_goal_score = 0.1
            if container != 1:
                general_score = 0


        #Get objects that are on top of goals:
        objects_ontop = []
        next_object_place = 0
        for i in range(len(self.state_object_list_ontop)):
            if i < next_object_place:
                continue
            num_obj_ontop = int(copy.deepcopy(self.state_object_list_ontop[i+1]))
            if self.state_object_list_ontop[i] in self.goals:
                for j in range(num_obj_ontop):
                    objects_ontop.append(self.state_object_list_ontop[i+2+j])
            next_object_place = i+1+num_obj_ontop+1
            #    break

        if len(self.all_suction_points) > 0:
            for i in range(0, self.all_suction_points.shape[0]):
                self.all_suction_points[i, self.param_suction-1] *= general_score
                if self.suction_object_list[i] not in self.goals:
                    self.all_suction_points[i, self.param_suction-1] *= no_goal_score #You do not want to consider this point if it has a different ID
                    if self.suction_object_list[i] in objects_ontop:
                        self.all_suction_points[i, self.param_suction-1] *= (no_goal_score+1.)/(2.*(no_goal_score+0.01)) #You do not want to consider this point if it has a different ID
                else:
                    index_visibility = (list(self.object_vision_list)).index(self.suction_object_list[i])
                    if self.object_visibility[index_visibility] < 0.1:
                        self.all_suction_points[i, self.param_suction-1] *= bad_visibility_score #You prefer objects with high visibility

                if self.failed_attempts_in_high_surface > 1:
                    condition_for_x = (self.all_suction_points[i, 0] < self.bin1_pose[0]-0.05)
                    if condition_for_x and container == 1 and not (self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0):  #condition suction point in wrong side
                        self.all_suction_points[i, self.param_suction-1] *= 0.1 #You prefer objects with high visibility
                #TODO_M: we could also consider what is the object that it is closest to it
                #TODO_M: we could also consider its visibility and objects on top

        if len(self.all_suction_side_points) > 0:
            for i in range(0, self.all_suction_side_points.shape[0]):
                self.all_suction_side_points[i, self.param_suction_side-1] *= general_score
                if self.suction_side_object_list[i] not in self.goals:
                    self.all_suction_side_points[i, self.param_suction_side-1] *= no_goal_score #You do not want to consider this point if it has a different ID
                    if self.suction_side_object_list[i] in objects_ontop:
                        self.all_suction_side_points[i, self.param_suction_side-1] *= (no_goal_score+1.)/(2.*(no_goal_score+0.01)) #You do not want to consider this point if it has a different ID
                else:
                    index_visibility = (list(self.object_vision_list)).index(self.suction_side_object_list[i])
                    if self.object_visibility[index_visibility] < 0.1:
                        self.all_suction_side_points[i, self.param_suction_side-1] *= bad_visibility_score #You prefer objects with high visibility


        if len(self.all_grasp_proposals) > 0:
            for i in range(0, self.all_grasp_proposals.shape[0]):
                self.all_grasp_proposals[i, self.param_grasping-1] *= general_score
                if self.grasp_object_list[i] not in self.goals:
                    self.all_grasp_proposals[i, self.param_grasping-1] *= no_goal_score #You do not want to consider this point if it has a different ID
                    if self.grasp_object_list[i] in objects_ontop:
                        self.all_grasp_proposals[i, self.param_grasping-1] *= (no_goal_score+1.)/(2.*(no_goal_score+0.01)) #You do not want to consider this point if it has a different ID
                else:
                    index_visibility = (list(self.object_vision_list)).index(self.grasp_object_list[i])
                    if self.object_visibility[index_visibility]  < 0.1:
                        self.all_grasp_proposals[i, self.param_grasping-1] *= bad_visibility_score #You prefer objects with high visibility
        if len(self.all_flush_proposals) > 0:
            for i in range(0, self.all_flush_proposals.shape[0]):
                self.all_flush_proposals[i, self.param_flush-1] *= general_score
                if self.flush_grasp_object_list[i] not in self.goals:
                    self.all_flush_proposals[i, self.param_flush-1] *= no_goal_score #You do not want to consider this point if it has a different ID
                    if self.flush_grasp_object_list[i] in objects_ontop:
                        self.all_flush_proposals[i, self.param_flush-1] *= (no_goal_score+1.)/(2.*(no_goal_score+0.01)) #You do not want to consider this point if it has a different ID
                else:
                    index_visibility = (list(self.object_vision_list)).index(self.flush_grasp_object_list[i])
                    if self.object_visibility[index_visibility] < 0.1:
                        self.all_flush_proposals[i, self.param_flush-1] *= bad_visibility_score #You prefer objects with high visibility

        return

    def read_interactive_file(self):
        with open(self.interactive_filename) as data_file:
                DATA = json.load(data_file)
        permutation = DATA['permutation']
        for i, x in enumerate(permutation):
            obj = self.objects[x]
            obj.label = DATA['label'][i]
            obj.container = DATA['container'][i]
            obj.w_dim = DATA['w_dim'][i]
            obj.pose = DATA['pose'][i]
            obj.theta = DATA['theta']
            obj.pos = DATA['pos'][i]

    ################
    ### BASELINE ###
    ################

    def interactive_add_object_get_best_container(self,objName):
        objName = str.lower(objName)
        self.objects.append(Item(container=0,is_goal=self.ini_is_goal))
        self.objects[-1].update_with_label(objName)
        best_container = self.PlacingPlanner.place_object_choosebin(-1)
        return best_container

    def pickingPreparation(self):
        if opt.interactive_filename is not '':
            self.jsonfilename = self.interactive_filename
            #self.enter_jsonfile()
            return
        DATA = {}
        DATA['label'] = []; DATA['container'] = []; DATA['w_dim'] = []; DATA['pose'] = []; DATA['theta'] = []; DATA['pos'] = [];
        self.placing_index_in_container = [];  #SS: store the ordering of placing
        #SS: get initial HM
        # e.db('[ PLANNER ] Getting height map from vision.')
        # SS: seems we don't need height map any more
        # if self.PlacingPlanner.visionType == 'real':
        #     self.PlacingPlanner.update_real_height_map()

        if self.visionType == 'real':
            restart_passive_vision = raw_input('Restart passive vision? [y/n]')
            while restart_passive_vision != 'n' and restart_passive_vision != 'y':
                restart_passive_vision = raw_input('Restart passive vision? [y/n]')
            restart_passive_vision = (restart_passive_vision == 'y')
            if restart_passive_vision:
                self.getPassiveVisionEstimate('restart', '', 0)
                e.db('[ PLANNER ] Restarting passive vision.')
                rospy.sleep(10)


        # AZ: Start interactive placing interface
        ipif = PlacingInterface()
        cv2.namedWindow("Interactive Placing")
        cv2.setMouseCallback("Interactive Placing", ipif.placing_interface_mouse_events)
        cv2.imshow("Interactive Placing", ipif.blankCanvas)

        # TODO: when start planner has 0 object in storage system ??
        e.db('Ready for placing objects.')
        previous_cmd_container = -1
        time_before_update = math.floor(time.time())

        num_objects_to_be_placed = 0
        if self.combined_task:
            num_objects_to_be_placed = self.num_combined_stow
        else:
            num_objects_to_be_placed = self.num_pick

        # Add object to planner on click
        ipif.suggestionFunction = self.interactive_add_object_get_best_container

        for j in range(num_objects_to_be_placed):

        # for j in range(len(self.objects)):
        #     #now only go through the list
        #     if self.objects[j].container == 0:
        #         continue
        #     print('Name of object #'+str(j)+': '+self.objects[j].label)
        #     input_object_id = j

        #     obj = self.objects[input_object_id]

            # suggeste best bin

            # Inform placing planner of selected object
            ipif.eventStep = 0
            # ipif.suggestedBin = obj.container
            # ipif.force_select_object(str(obj.label))

            # AZ: Select which bin and press enter to confirm
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key < 255:
                    print(key)
                    if ipif.eventStep == 1:
                        # If number keys 1 - 3 are pressed
                        if key >= 49 and key <= 51:
                            key = key-48
                            ipif.numberCanvas = ipif.selectionCanvas.copy()
                            ipif.suggestedBin = key
                            cv2.putText(ipif.numberCanvas,str(ipif.suggestedBin),(ipif.objGridMidX[ipif.selectedObjInd]-50,ipif.objGridMidY[ipif.selectedObjInd]+65), cv2.FONT_HERSHEY_SIMPLEX, 5,(0,255,0),5)
                            cv2.imshow("Interactive Placing", ipif.numberCanvas)

                        # If enter is pressed
                        if key == 13:
                            ipif.numObjSelected += 1
                            ipif.blankCanvas[0:200,1500:1700] = np.ones((200,200,3), np.uint8)*255;
                            cv2.putText(ipif.blankCanvas,str(num_objects_to_be_placed - ipif.numObjSelected),(1500,150), cv2.FONT_HERSHEY_SIMPLEX, 5,(255,0,0),5)
                            cv2.imshow("Interactive Placing", ipif.blankCanvas)
                            root = tk.Tk()
                            root.withdraw()
                            tkMessageBox.showwarning("Confirm","Is {} placed in bin {} and is Shuran's hand out?".format(ipif.cmptnObjList[ipif.selectedObjInd],ipif.suggestedBin))
                            # print("{} placed in bin {}".format(ipif.cmptnObjList[ipif.selectedObjInd],ipif.suggestedBin))
                            break

            obj = self.objects[-1]
            obj.container = ipif.suggestedBin;
            e.db(obj.label, 'go in bin', obj.container, 'with unknown pose')

            # #SS: reset the bin id use user input only do it once directly reset it
            # if self.visionType == 'real':
            #     do_you_like = raw_input('Is it good?[y/n]')
            #     while do_you_like != 'n' and do_you_like != 'y':
            #         do_you_like = raw_input('Is it good?[y/n]')
            #     do_you_like = (do_you_like == 'y')
            #     if not do_you_like:
            #         bin_num = raw_input('What bin is the object placed? [1/2/3]')
            #         while bin_num != '1' and bin_num != '2' and bin_num != '3':
            #               bin_num = raw_input('What bin is the object placed? [1/2/3]')
            #         bin_num = int(bin_num)
            #         obj.container = bin_num
            #         e.db(obj.label, 'go in bin', obj.container, 'with unknown pose')

            #Update its position and add to the map
            #self.PlacingPlanner.update_pose_placed_obj(input_object_id)
            self.initial_objects_bins[obj.container].append(obj.label)
            self.shelfObj = range(len(self.toteObj), len(self.objects))
            #Show result of placing:
            self.PlacingPlanner.show_placing_decision(b = obj.container-1, theta = obj.theta, hm_pre_pub= self.hm_pre_pub, hm_post_pub= self.hm_post_pub, score_pub= self.score_pub)
            DATA['label'].append(obj.label); DATA['container'].append(obj.container); DATA['w_dim'].append(obj.w_dim);
            DATA['pose'].append(obj.pose); DATA['theta'].append(obj.theta); DATA['pos'].append(obj.pos);

            # if self.PlacingPlanner.visionType == 'real':
            #     raw_input('[ PLANNER ] Object ' + obj.label + ' placed in ' + str(obj.container) +' (Enter)')

            if self.visionType == 'real':
                # SS wait for previous update finished
                if (j>0 and previous_cmd_container > -1) :
                    self.request_passive_vision_wait(previous_cmd_container)
                # SS: send update command and recod this update for later checking
                previous_cmd_container = obj.container
                add_command = '{} add {} '.format(obj.container,obj.label)
                self.getPassiveVisionEstimate('update state', add_command, obj.container)

        # AZ: Delete placing interface window
        cv2.destroyAllWindows()
        if self.visionType == 'real':
            self.request_passive_vision_wait(previous_cmd_container)

        # Placing is done
        self.json_output(interactive_file = True)
        outfilename = os.environ['ARC_BASE']+'/input/interactive_placing.json'
        with open(outfilename, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        raw_input('[ PLANNER ] Placing is done. Press enter to continue...')
        return

    def pickingPreparationOld(self):
        if opt.interactive_filename is not '':
            self.read_interactive_file()
            return
        permutation = np.random.permutation(self.shelfObj)
        DATA = {}
        DATA['permutation'] = list(permutation)
        DATA['label'] = []; DATA['container'] = []; DATA['w_dim'] = []; DATA['pose'] = []; DATA['theta'] = []; DATA['pos'] = [];
        for j in permutation:
            #Update HM
            if self.PlacingPlanner.visionType == 'real': #TODO: is this the best place?
                self.PlacingPlanner.update_real_height_map()
            #Get best z position
            obj = self.objects[j]
            e.db('We are placing :', obj.label)
            do_you_like = False
            while not do_you_like:
                best_score = 1000000
                best_ori = 0
                best_z = 0
                best_Scores = None
                best_container = 1
                for z in range(3):
                    if obj.placeability[z] == 0:
                        continue
                    #Swap z dimension
                    obj.w_dim = copy.deepcopy(obj.dim)
                    obj.w_dim[2], obj.w_dim[z] = obj.w_dim[z], obj.w_dim[2]
                    #for i in range(self.num_bins):
                        #obj.container = i+1
                        #Find best place object given dimension changed
                    score_for_bin = self.PlacingPlanner.place_object_local_best(int(j)) #Change container and pos placedObj
                    '''
                    print 'obj.container: ', obj.container
                    print 'obj.pos: ', obj.pos
                    print 'obj.dim: ', obj.dim
                    print 'obj.w_dim: ', obj.w_dim
                    print 'score_for_bin: ', score_for_bin
                    print 'obj.theta: ', obj.theta
                    '''
                    if best_score > int(score_for_bin):
                        best_score = copy.deepcopy(score_for_bin)
                        best_z = copy.deepcopy(z)
                        best_container = copy.deepcopy(obj.container)
                        best_pos = copy.deepcopy(obj.pos)
                        best_theta = copy.deepcopy(obj.theta)
                #Swap z dimension
                obj.w_dim = copy.deepcopy(obj.dim)
                obj.w_dim[2], obj.w_dim[best_z] = obj.w_dim[best_z], obj.w_dim[2]
                #Assign best results to obj: container, pos, theta
                obj.container = best_container
                obj.pos = best_pos
                obj.theta = best_theta
                e.db('--------PLANNER DECISION------')
                e.db('------------------------------')
                e.db('obj.pos: ', obj.pos, 'obj.dim: ', obj.dim, 'obj.w_dim: ', obj.w_dim, ' best_z:', best_z, ' best_score:', best_score)
                #Update its position in world frame and add to the map
                self.PlacingPlanner.update_pose_placed_obj(int(j))
                e.db(obj.label, 'go in bin', obj.container, 'in the pose', obj.pose, 'or', obj.pos, 'in the tote frame and theta', obj.theta)
                self.PlacingPlanner.show_placing_decision(b = obj.container-1, theta = obj.theta, hm_pre_pub= self.hm_pre_pub, hm_post_pub= self.hm_post_pub, score_pub= self.score_pub)
                if self.visionType == 'real':
                    do_you_like = raw_input('Is it good?[y/n]')
                    while do_you_like != 'n' and do_you_like != 'y':
                        do_you_like = raw_input('Is it good?[y/n]')
                    do_you_like = (do_you_like == 'y')
                    if not do_you_like:
                        bin_num = raw_input('What bin should I try?[1/2/3]')
                        while bin_num != '1' and bin_num != '2' and bin_num != '3':
                            bin_num = raw_input('What bin should I try?[1/2/3]')
                        bin_num = int(bin_num)
                        self.PlacingPlanner.available = [False, False, False, True, True, True]
                        self.PlacingPlanner.available[bin_num-1] = True
                        obj.container = bin_num
                else:
                    do_you_like = True
            #Updating a json file
            DATA['label'].append(obj.label); DATA['container'].append(obj.container); DATA['w_dim'].append(obj.w_dim);
            DATA['pose'].append(obj.pose); DATA['theta'].append(obj.theta); DATA['pos'].append(obj.pos);
            if self.visionType == 'real':
                raw_input('Next object')
            self.clean_bin(obj.container) #Update clean state because hands went into the robot
            if self.visionType == 'real':
                add_command = '{} add {} {} {} {}'.format(obj.container,obj.label, obj.pose[0], obj.pose[1], obj.pose[2])
                self.getPassiveVisionEstimate(add_command, obj.container)
        outfilename = os.environ['ARC_BASE']+'/input/interactive_placing.json'
        with open(outfilename, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        return

    def baselinePreparation(self):
        pass

    def baseline_draw(self):
        '''Baseline drawing operation. Compute the best suction and
        grasping score, then run whichever is best'''
        e.db('Objects in the tote: ', self.toteObjNames())
        #Get Best Suction and Grasping Points
        self.getBestSuctionPoint()
        self.getBestSuctionSidePoint()
        self.getBestGraspingPoint()
        #Execute the best primitive
        doing_suction = 'suction-tote' in self.primitives and self.suction_point is not None and self.decideDrawPrimitive()
        if 'grasp-tote' not in self.primitives or doing_suction:
            self.best_primitive = 'suction-tote'
            e.db('-----------------------------')
            self.webpage.updateValue('SuctionScore', self.suction_score)
            self.webpage.updateValue('SuctionSideScore', self.suction_side_score)
            #self.suction_side = True
            if self.suction_side:
                self.suction_point = copy.deepcopy(self.suction_side_point)
                self.suction_score = copy.deepcopy(self.suction_side_score)
                e.db('      RUNNING SUCTION SIDE    ')
                self.webpage.updateValue('RobotAction', 'Side Suction')
            else:
                e.db('        RUNNING SUCTION      ')
                self.webpage.updateValue('RobotAction', 'Suction')
            e.db('-----------------------------')
            self.run_suction()
        else:
             self.best_primitive = 'grasp-tote'
             e.db('-----------------------------')
             e.db('       RUNNING GRASPING      ')
             e.db('-----------------------------')
             self.webpage.updateValue('RobotAction', 'Grasping')
             self.run_grasping()
        self.previous_primitive = copy.deepcopy(self.best_primitive)

    def random_draw(self):
        '''Random drawing operation. Compute best suction and grasping scores
        Randomly run either operation, given that there is a suction or grasp
        point (respectively)'''
        e.db('Objects in the tote: ', self.toteObjNames())
        #Get Best Suction and Grasping Points
        self.getBestSuctionPoint()
        self.getBestGraspingPoint()
        #Execute the best primitive
        doing_suction = 'suction-tote' in self.primitives and (np.random.rand() > 0.5 or self.grasp_point is None) and self.decideDrawPrimitive()
        if 'grasp-tote' not in self.primitives or doing_suction:
            self.best_primitive = 'suction-tote'
            e.db('-----------------------------')
            e.db('        RUNNING SUCTION      ')
            e.db('-----------------------------')
            self.run_suction()
        else:
            self.best_primitive = 'grasp-tote'
            e.db('-----------------------------')
            e.db('        RUNNING GRASPING     ')
            e.db('-----------------------------')
            self.run_grasping()
        self.previous_primitive = copy.deepcopy(self.best_primitive)

    def baseline_logging(self):
        return False #This function is asked whether to stop the experiment, in baseline we don't stop

    def decideDrawPrimitive(self):
        #Returns True when suction is selected as the best primitive
        self.suction_side = False
        if self.grasp_point is None:
            return True
        if self.suction_point is None:
            return False
        suction_score = copy.deepcopy(self.suction_score)
        suction_side_score = copy.deepcopy(self.suction_side_score)
        grasp_score = copy.deepcopy(self.grasp_score)
        if len(self.toteObj) >= 10:
            grasp_score *= 0.5
            suction_side_score *= 0.5
        else:
            grasp_score *= 0.95
            suction_side_score *= 0.90
        if len(self.bad_suction_points) > 1:
            suction_score *= 0.5
            if len(self.bad_suction_points) > 3:
                suction_score *= 0.5
        if len(self.bad_suction_side_points) > 1:
            suction_side_score *= 0.5
            if len(self.bad_suction_side_points) > 3:
                suction_side_score *= 0.5
        if len(self.bad_grasping_points) > 1:
            grasp_score *= 0.5
            if len(self.bad_grasping_points) > 3:
                grasp_score *= 0.5
        e.db('The suction_score considered is: ', suction_score, ' the suction_side_score is: ', suction_side_score, ' the grasp_score considered is: ', grasp_score)
        self.webpage.updateValue('SuctionScore', suction_score)
        self.webpage.updateValue('SuctionSideScore', suction_side_score)
        self.webpage.updateValue('GraspingScore', grasp_score)

        if suction_score < suction_side_score:
            self.suction_side = True
            suction_score = suction_side_score
        return suction_score > grasp_score

    def getBestSuctionPoint(self, container = None):
        '''Get the best suction point and correct the normal orientation'''
        self.suction_point, self.suction_score, self.suction_id, self.suction_id_confidence = self.GetSuctionPoints(number_points=1, container=container)
        self.webpage.updateValue('SuctionScore', self.suction_score)
        if self.suction_point is None:
            e.db('THERE ARE NO SUCTION POINTS')
            self.webpage.updateValue('AreSuctionPts', False)
            return
        self.webpage.updateValue('AreSuctionPts', True)
        # Forcefully aim 2cm below suction point
        self.suction_point[2] = self.suction_point[2] - 0.02
        # TODO: now normal is considered fixed at 0,0,1
        self.suction_point = list(self.suction_point)
        #visualize_suction_point(self.proposal_viz_array_pub, self.suction_point, 'b')
        self.suction_point[3] = 0
        self.suction_point[4] = 0
        self.suction_point[5] = 1
        e.db('Best suction point:', self.suction_point, 'with score: ', self.suction_score, ' and id ', self.suction_id, 'in bin: ', container, 'and with confidence:', self.suction_id_confidence)

    def getBestSuctionSidePoint(self, container = None):
        '''Get the best suction side oint and correct the normal orientation'''
        self.suction_side_point, self.suction_side_score, self.suction_side_id, self.suction_side_id_confidence = self.GetSuctionSidePoints(number_points=1, container=container)
        if self.suction_side_point is None:
            e.db('THERE ARE NO SUCTION SIDE POINTS')
            self.webpage.updateValue('AreSuctionPts', False)
            return
        self.webpage.updateValue('AreSuctionPts', True)
        # Forcefully aim 2cm below suction point
        self.suction_side_point[2] = self.suction_side_point[2] - 0.03
        self.suction_side_point = list(self.suction_side_point)
        #visualize_suction_point(self.proposal_viz_array_pub, self.suction_side_point, 'db')
        e.db('Best suction side point:', self.suction_side_point, 'with score: ', self.suction_side_score, ' and id ', self.suction_side_id, 'in bin: ', container, 'and with confidence:', self.suction_side_id_confidence)


    def getBestGraspingPoint(self, container = None):
        '''Get the best grasp point considering both grasp and flush.
            Remove those that are in collision.'''
        if container == None:
            container = self.tote_ID

        self.GetGraspPoints(num_points=100,container = container)
        #If no grasp or flush point available:
        self.execution_possible = False
        if self.num_pick_proposals == 0:
            self.grasp_point = None
            self.grasp_score = 0
            self.webpage.updateValue('AreGraspPts', False)    
            return

        self.webpage.updateValue('AreGraspPts', True)
        #Find the best point that it is not in collision:
        num_attempts = 0
        num_it = 0
        while not self.execution_possible and num_attempts < 20 and num_it < self.num_pick_proposals: #Each time try at most 20 attempts
            grasp_point = copy.deepcopy(list(self.all_pick_proposals[num_it][:]))
            try:
                pass
                #visualize_grasping_proposals(self.proposal_viz_array_pub, np.array([grasp_point]), True)
            except:
                e.db('Processing flush grasp not visualizing')
            num_it += 1
            #If we already know it is a bad point, we do not try it again
            if grasp_point in self.bad_grasping_points:
                return
            #Check if in collision
            num_attempts += 1
            if len(grasp_point) == self.param_flush:
                checked_output = flush_grasp(objInput=grasp_point, listener=self.listener, br=self.br, isExecute=False,
                                             binId=container, flag=0, withPause=False, viz_pub=self.viz_array_pub)
            else:
                checked_output = grasp(objInput=grasp_point, listener=self.listener, br=self.br, isExecute=False,
                                       binId=container, flag=0, withPause=False, viz_pub=self.viz_array_pub)
            if checked_output['execution_possible']:
                self.grasp_score = copy.deepcopy(self.all_pick_scores[num_it-1])
                self.grasp_point = copy.deepcopy(grasp_point)
                self.webpage.updateValue('GraspingScore', self.grasp_score)
                self.grasp_id = None
                if len(grasp_point) == self.param_flush:
                    for i in range(len(self.all_flush_proposals)):
                        if grasp_point == self.all_flush_proposals[i,0:self.param_flush].tolist():
                            self.grasp_id = self.flush_grasp_object_list[i]
                            self.grasp_id_confidence = self.flush_grasp_object_confidence[i]
                else:
                    for i in range(len(self.all_grasp_proposals)):
                        if grasp_point == self.all_grasp_proposals[i,0:self.param_grasping].tolist():
                            self.grasp_id = self.grasp_object_list[i]
                            self.grasp_id_confidence = self.grasp_object_confidence[i]
                e.db('Best grasp point:', grasp_point, ' with score: ', self.grasp_score, ' with id: ', self.grasp_id, 'in bin: ', container, ' and with id confidence: ', self.grasp_id_confidence)
                self.webpage.updateValue('GraspingScore', self.grasp_score)
                return
            if grasp_point is not None:
                self.bad_grasping_points.append(grasp_point)
                self.bad_grasping_times.append(-1)
        e.db('NONE OF THE GRASPING POINTS WORK')
        self.grasp_point = None
        self.grasp_score = 0
        self.grasp_id = 'Null'
        e.db('Best grasp point:', grasp_point, ' with score: ', self.grasp_score, ' with id: ', self.grasp_id, 'in bin: ', container)
        self.webpage.updateValue('GraspingScore', self.grasp_score)
        return


    def call_passive_vision(self, bin_id=0):
        '''Call passive vision, log file paths, and return suction points and grasp proposals'''
        e.db('Calling passive vision system for suction and grasp proposals. The bin considered is: ', bin_id)
        with Timer('getPassiveVisionEstimate ' + 'request hm sg %d' % bin_id ):
            self.passive_vision_state = self.request_passive_vision_wait(bin_id)

        e.db('Received suction and grasp proposals.')
        #Suction
        self.all_suction_points = np.asarray(self.passive_vision_state.suction_points)
        self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/self.param_suction, self.param_suction)
        self.suction_object_list = np.asarray(self.passive_vision_state.suction_object_list)
        self.suction_object_confidence = np.asarray(self.passive_vision_state.suction_object_confidence)
        #visualize_suction_points(self.proposal_viz_array_pub, self.all_suction_points, colorid='g')
        #Suction side
        self.all_suction_side_points = np.asarray(self.passive_vision_state.suctionside_points)
        self.all_suction_side_points = self.all_suction_side_points.reshape(len(self.all_suction_side_points)/self.param_suction_side, self.param_suction_side)
        self.suction_side_object_list = np.asarray(self.passive_vision_state.suctionside_object_list)
        self.suction_side_object_confidence = np.asarray(self.passive_vision_state.suctionside_object_confidence)
        #visualize_suction_points(self.proposal_viz_array_pub, self.all_suction_side_points, colorid='dg')
        #Grasp
        self.all_grasp_proposals = np.asarray(self.passive_vision_state.grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/self.param_grasping, self.param_grasping)
        self.grasp_object_list = np.asarray(self.passive_vision_state.grasp_object_list)
        self.grasp_object_confidence = np.asarray(self.passive_vision_state.grasp_object_confidence)
        #visualize_grasping_proposals(self.proposal_viz_array_pub, self.all_grasp_proposals, False)
        #FlushGrasp
        self.all_flush_proposals = np.asarray(self.passive_vision_state.flush_grasp_proposals)
        self.all_flush_proposals = self.all_flush_proposals.reshape(len(self.all_flush_proposals)/self.param_flush, self.param_flush)
        self.flush_grasp_object_list = np.asarray(self.passive_vision_state.flush_grasp_object_list)
        self.flush_grasp_object_confidence = np.asarray(self.passive_vision_state.flush_grasp_object_confidence)
        #visualize_flush_proposals(self.proposal_viz_array_pub, self.all_flush_proposals, bin_id, self.listener, self.br, False)

        #If doing picking, we need to filter them given the goal objects
        if self.task == 'picking':
                self.object_visibility = np.asarray(self.passive_vision_state.state_object_visibility)
                self.object_vision_list = np.asarray(self.passive_vision_state.state_object_list)
                self.state_object_list_ontop = np.asarray(self.passive_vision_state.state_object_list_ontop)
                self.pickingProposalsFilter(container = bin_id)
        # Log saved image files and camera intrinsics files
        if self.experiment:
            self.log_file.write('passive_vision_files {}\n'.format(self.passive_vision_state.file_paths[0]))

        #Sorting all points:  #TODO: ask vision if this is still needed
        suction_permutation = self.all_suction_points[:,self.param_suction-1].argsort()[::-1]
        self.all_suction_points = self.all_suction_points[suction_permutation]
        self.suction_object_list = self.suction_object_list[suction_permutation]
        self.suction_object_confidence = self.suction_object_confidence[suction_permutation]

        suction_side_permutation = self.all_suction_side_points[:,self.param_suction_side-1].argsort()[::-1]
        self.all_suction_side_points = self.all_suction_side_points[suction_side_permutation]
        self.suction_side_object_list = self.suction_side_object_list[suction_side_permutation]
        self.suction_side_object_confidence = self.suction_side_object_confidence[suction_side_permutation]

        grasp_permutation = self.all_grasp_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]
        self.grasp_object_list = self.grasp_object_list[grasp_permutation]
        self.grasp_object_confidence = self.grasp_object_confidence[grasp_permutation]

        flush_permutation = self.all_flush_proposals[:,self.param_flush-1].argsort()[::-1]
        self.all_flush_proposals = self.all_flush_proposals[flush_permutation]
        self.flush_grasp_object_list = self.flush_grasp_object_list[flush_permutation]
        self.flush_grasp_object_confidence = self.flush_grasp_object_confidence[flush_permutation]

    def CallActiveVision(self, bin_id, primitive_id, primitive_id_confidence):
        '''Call Active Vision and predict the name of the object. Add log
        @param bin_id Bin to perform vision on
        @return predicted_object_name Guesses name of object'''
        if self.task == 'stowing':
            bin_id = 1
        if self.in_simulation:
            return self.callFakeActiveVision(bin_id)
        e.db('Calling active vision system to recognize object in gripper.')
        getActiveVisionRecognition = rospy.ServiceProxy('/active_vision/recognize', active_vision.srv.recognition)
        # active vision takes in two lists now, first one is smaller list, second one is full object list
        while True:
            try:
                if self.task == 'stowing':
                    active_vision_data = getActiveVisionRecognition(bin_id, self.weight_info[self.tote_ID]['weights']/1000.0, self.containerObjNames(self.tote_ID), self.initial_objects_bins[self.tote_ID], primitive_id, primitive_id_confidence)
                else:
                    active_vision_data = getActiveVisionRecognition(bin_id, self.weight_info[bin_id]['weights']/1000.0, self.containerObjNames(bin_id), self.initial_objects_bins[bin_id], primitive_id, primitive_id_confidence)
                break
            except Exception as err:
                print err
                rospy.sleep(0.1)
        e.db('object_confidence_with_weights: ', active_vision_data.object_confidence_with_weights)
        e.db('object_name: ', active_vision_data.prediction_full_list)
        # highest_confidence = 0
        # predicted_object_idx = 0
        # for i in range(0, len(active_vision_data.object_confidence_with_weights)):
        #     if active_vision_data.object_confidence_with_weights[i] > highest_confidence:
        #         highest_confidence = active_vision_data.object_confidence_with_weights[i]
        #         predicted_object_idx = i
        # predicted_object_name = active_vision_data.object_name[predicted_object_idx]
        prediction_full_list = active_vision_data.prediction_full_list;
        prediction_small_list = active_vision_data.prediction_small_list;
        predicted_object_name = prediction_full_list;
        # print(predicted_object_name)
        # print(prediction_small_list)
        # print(not prediction_small_list) # prints true if prediction_small_list is empty
        if len(prediction_small_list) == 0:
            prediction_small_list = predicted_object_name
        if self.experiment:
            self.log_file.write('active_vision_files {}\n'.format(active_vision_data.file_paths[0]))
            self.log_file.write('active_vision_prediction {}\n'.format(predicted_object_name))
        return predicted_object_name, prediction_small_list, active_vision_data.rot_bbox_info, active_vision_data.bbox_info

    def getCompetitionObj(self):
        cmptnItemDataPath = os.environ['ARCDATA_BASE'] + '/itemdata'
        cmptnObjList = []
        for dirname, cmptnObjNames, filenames in os.walk(cmptnItemDataPath):
            for cmptnObjName in cmptnObjNames:
                if cmptnObjName != 'Empty':
                    cmptnObjList.append(cmptnObjName.lower())
        cmptnObjList.sort()
        return cmptnObjList
        
    def json_output(self, interactive_file = False):
        if interactive_file:
            self.interactivefilename = os.environ['ARC_BASE']+'/input/interactive_result.json'
        if self.task == 'stowing':
            self.outfilename = os.environ['ARCDATA_BASE']+'/stow_result/stow_result.json'
            self.outfilename_in_output = os.environ['ARC_BASE']+'/output/stow_result.json'
            #Add all objects in bin 1
            self.outfilename_complete = os.environ['ARCDATA_BASE']+'/stow_result_complete/stow_result_complete.json'
            self.outfilename_complete_in_output = os.environ['ARC_BASE']+'/output/stow_result_complete.json'
        else:
            self.outfilename = os.environ['ARCDATA_BASE']+'/pick_result/pick_result.json'
            self.outfilename_in_output = os.environ['ARC_BASE']+'/output/pick_result.json'
        DATA = {}
        DATA['bins'] = []
        for x in range(self.num_bins):
            DATA['bins'].append({'bin_id': chr(ord('A')+x), 'contents': self.shelfObjNamesGivenBin(x+1) })
        DATA['tote'] = {}
        DATA['tote']['contents'] = self.toteObjNames()
        DATA['boxes'] = []
        if self.task == 'picking':
            for x in range(5): #boxes
                DATA['boxes'].append({'bin_id': self.box_id[x], 'contents': self.shelfObjNamesGivenBox(x+4) })
        with open(self.outfilename, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(self.outfilename_in_output, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        if interactive_file:
            competition_obj = self.getCompetitionObj()
            bins_obj = []
            #bins_obj = [obj for obj in bin_i['contents'] for bin_i in DATA['bins']]
            for bin_i in DATA['bins']:
                bins_obj.extend(bin_i['contents'])
            DATA['tote']['contents'] = [i for i in competition_obj if i not in bins_obj]
            with open(self.interactivefilename, 'w') as outfile:
                json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(self.outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        if self.task == 'stowing':
            for i, x in enumerate(self.toteObj):
                obj = self.objects[x]
                if obj in self.list_objects_in_bin_1:
                    DATA['bins'][0]['contents'].append(obj.label)
                else:
                    DATA['bins'][2]['contents'].append(obj.label)
            DATA['tote']['contents'] = []
            with open(self.outfilename_complete, 'w') as outfile:   # save every iteration seperately in case the result got overwritten
                json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
            with open(self.outfilename_complete_in_output, 'w') as outfile:   # save every iteration seperately in case the result got overwritten
                json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
            with open(self.outfilename_complete + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
                json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)

    def GetSuctionPoints(self, number_points, container = None):
        ''' Call vision and do some math on it (?) to return maximally
        number_points number of suction points
        @param number_points Number of suction points to return
        @return suction_points Suction points with their scores'''
        if container == None: #Stowing task
            container = self.tote_ID
        param_suction = self.param_suction
        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](container)
        try_get_points = 0
        while (len(self.all_suction_points)+len(self.all_suction_side_points)+len(self.all_grasp_proposals)+len(self.all_flush_proposals)) == 0 and try_get_points < 1:  #Try to find suction points 2 times
            try_get_points += 1
            self.getPassiveVisionEstimate('update hm sg', '', container)
            self.passiveVisionTypes[self.visionType](container)
        if try_get_points >= 1:
            e.db('There are no proposals at all')
            self.times_no_points_in_container[container] = self.times_no_points_in_container[container] + 2
        else:
            self.times_no_points_in_container[container] = 0
        #I already do the next 2 lines in passiveVision
        #self.all_suction_points = np.asarray(self.all_suction_points)
        #self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/param_suction,param_suction)

        # NOTE: REMOVE THIS WHEN SUCTION DIRECTION IS CONTROLLABLE
        # Only keep suction points that are valid for suction down (with surface normals close to gravity)

        if self.visionType == 'real' and len(self.all_suction_points) > 0: #You just want to remove points in real
            updown_suction_points_indices = []
            e.db(SUCTIONPOINTS=self.all_suction_points)
            for i in range(0, self.all_suction_points.shape[0]):
                angle2gravity = np.rad2deg(np.arctan2(la.norm(np.cross([self.all_suction_points[i, 3], self.all_suction_points[i, 4], self.all_suction_points[i, 5]], [0, 0, 1])),
                                np.dot([self.all_suction_points[i, 3], self.all_suction_points[i, 4], self.all_suction_points[i, 5]], [0, 0, 1])))
                if angle2gravity < 20: # If angle to gravity direction is less than 20 degrees
                    updown_suction_points_indices.append(i)
            updown_suction_points = np.zeros((len(updown_suction_points_indices), param_suction))
            updown_suction_ids = ['NULL']*len(updown_suction_points_indices)
            updown_suction_confidence = [0]*len(updown_suction_points_indices)
            for i in range(0, len(updown_suction_points_indices)):
                updown_suction_points[i] = self.all_suction_points[updown_suction_points_indices[i], :]
                updown_suction_ids[i] = self.suction_object_list[updown_suction_points_indices[i]]
                updown_suction_confidence[i] = self.suction_object_confidence[updown_suction_points_indices[i]]
            if len(updown_suction_points) > 0:
                self.all_suction_points = updown_suction_points
                self.suction_object_list = updown_suction_ids
                self.suction_object_confidence = updown_suction_confidence

            ## Filter out outdated bad_suction_point
            self.bad_suction_points, self.bad_suction_times = self.remove_old_points(self.bad_suction_points, self.bad_suction_times, 60*3)

            ## Remove suction points that are repeated:
            not_bad_suction_points_indices = []
            for i in range(0, self.all_suction_points.shape[0]):
                is_bad = False
                suction_point = list(self.all_suction_points[i, 0:param_suction-1])
                for j,bad_point in enumerate(self.bad_suction_points):
                    if np.linalg.norm(np.array(bad_point[0:2]) - np .array(suction_point[0:2]))< 0.025: #only looking position, not angle
                        is_bad = True
                        #e.db('..............................................................................................')
                        #e.db('Removed suction point: ', suction_point[0:2], ' because of previous bad point: ', bad_point[0:2])
                        break
                if not is_bad:
                    not_bad_suction_points_indices.append(i)
            e.db('There are ', len(self.bad_suction_points), ' bad_suction_points ',': ', self.bad_suction_points)
            not_bad_suction_points = np.zeros((len(not_bad_suction_points_indices), param_suction))
            not_bad_suction_ids = ['NULL']*len(not_bad_suction_points_indices)
            not_bad_suction_confidence = [0]*len(not_bad_suction_points_indices)
            for i in range(0, len(not_bad_suction_points_indices)):
                not_bad_suction_points[i] = self.all_suction_points[not_bad_suction_points_indices[i], :]
                not_bad_suction_ids[i] = self.suction_object_list[not_bad_suction_points_indices[i]]
                not_bad_suction_confidence[i] = self.suction_object_confidence[not_bad_suction_points_indices[i]]
            if len(not_bad_suction_points) > 0:
                self.all_suction_points = not_bad_suction_points
                self.suction_object_list = not_bad_suction_ids
                self.suction_object_confidence = not_bad_suction_confidence

        if self.visionType == 'virtual':
            self.suction_object_list = ['Null']*len(self.all_suction_points)
            self.suction_object_confidence = [0]*len(self.all_suction_points)

        if len(self.all_suction_points) == 0:
            return None, 0, 'Null', 0  #Return nothing

        ### Looking for the second best suction point:
        self.second_best_suction_point = copy.deepcopy(self.all_suction_points[0, 0:param_suction-1])
        if self.all_suction_points.shape[0] > 1:
            for i in range(1, self.all_suction_points.shape[0]):
                best_suction_point = list(self.all_suction_points[0, 0:param_suction-1])
                second_best_suction_point = list(self.all_suction_points[i, 0:param_suction-1])
                if np.linalg.norm(np.array(second_best_suction_point[0:2]) - np .array(best_suction_point[0:2]))> 0.15: # If they are more than 0.15cm apart
                    second_best_index = i
                    self.second_best_suction_point = copy.deepcopy(self.all_suction_points[i, 0:param_suction-1])
                    break
        e.db('second_best: ', self.second_best_suction_point) #, 'with index: ', second_best_index)
        if number_points == 1:
            #print self.all_suction_points
            return self.all_suction_points[0, 0:param_suction-1], self.all_suction_points[0, param_suction-1], self.suction_object_list[0], self.suction_object_confidence[0]
        return self.all_suction_points[:, 0:param_suction-1], self.all_suction_points[:, param_suction-1], self.suction_object_list, self.suction_object_confidence

    def GetSuctionSidePoints(self, number_points, container = None):

        ''' Call vision and do some math on it (?) to return maximally
        number_points number of suction points
        @param number_points Number of suction points to return
        @return suction_points Suction points with their scores'''
        if container == None: #Stowing task
            container = self.tote_ID
        param_suction_side = self.param_suction_side
        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](container)


        if self.visionType == 'real' and len(self.all_suction_side_points) > 0: #You just want to remove points in real

            ## Filter out outdated bad_suction_point
            self.bad_suction_side_points, self.bad_suction_side_times = self.remove_old_points(self.bad_suction_side_points, self.bad_suction_side_times, 60*3)

            ## Remove suction points that are repeated:
            not_bad_suction_side_points_indices = []
            for i in range(0, self.all_suction_side_points.shape[0]):
                is_bad = False
                suction_side_point = list(self.all_suction_side_points[i, 0:param_suction_side-1])
                for j,bad_point in enumerate(self.bad_suction_side_points):
                    if np.linalg.norm(np.array(bad_point[0:2]) - np .array(suction_side_point[0:2]))< 0.03: #only looking position, not angle
                        is_bad = True
                        #e.db('..............................................................................................')
                        #e.db('Removed suction side point: ', suction_side_point[0:2], ' because of previous bad point: ', bad_point[0:2])
                        break
                if not is_bad:
                    not_bad_suction_side_points_indices.append(i)
            not_bad_suction_side_points = np.zeros((len(not_bad_suction_side_points_indices), param_suction_side))
            not_bad_suction_side_ids = ['NULL']*len(not_bad_suction_side_points_indices)
            not_bad_suction_side_confidence = [0]*len(not_bad_suction_side_points_indices)
            e.db('There are ', len(self.bad_suction_side_points), ' bad_suction_side_points ',': ', self.bad_suction_side_points)
            for i in range(0, len(not_bad_suction_side_points_indices)):
                not_bad_suction_side_points[i] = self.all_suction_side_points[not_bad_suction_side_points_indices[i], :]
                not_bad_suction_side_ids[i] = self.suction_side_object_list[not_bad_suction_side_points_indices[i]]
                not_bad_suction_side_confidence[i] = self.suction_side_object_confidence[not_bad_suction_side_points_indices[i]]
            if len(not_bad_suction_side_points) > 0:
                self.all_suction_side_points = not_bad_suction_side_points
                self.suction_side_object_list = not_bad_suction_side_ids
                self.suction_side_object_confidence = not_bad_suction_side_confidence

        if self.visionType == 'virtual':
            self.suction_side_object_list = ['Null']*len(self.all_suction_side_points)
            self.suction_side_object_confidence = [0]*len(self.all_suction_side_points)

        if len(self.all_suction_side_points) == 0:
            return None, 0, 'Null', 0  #Return nothing
        if number_points == 1:
            return self.all_suction_side_points[0, 0:param_suction_side-1], self.all_suction_side_points[0, param_suction_side-1], self.suction_side_object_list[0], self.suction_side_object_confidence[0]
        return self.all_suction_side_points[:, 0:param_suction_side-1], self.all_suction_side_points[:, param_suction_side-1], self.suction_side_object_list, self.suction_side_object_confidence

    def remove_old_points(self, points, times, limit):
        new_points = []
        new_times = []
        for j, point in enumerate(points):
            if time.time() - times[j] < limit: #self.bad_suction_times[j] < time.time() + 60*3: #We only care about things that happened in the last 3 minutes
                new_points.append(point)
                new_times.append(times[j])
        return new_points, new_times

    def GetGraspPoints(self, num_points, container = None):
        '''Call passive vision to get grasp points.'''
        if container == None:
            container = self.tote_ID

        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](container)
        if self.visionType == 'virtual':
            self.grasp_object_list = ['Null']*len(self.all_grasp_proposals)
            self.grasp_object_confidence = ['Null']*len(self.all_grasp_proposals)
            self.flush_grasp_object_list = ['Null']*len(self.all_flush_proposals)
            self.flush_grasp_object_confidence = ['Null']*len(self.all_flush_proposals)
        #Add grasp proposals if possible
        e.db('There are ', len(self.bad_grasping_points), ' bad_grasping_points ',': ', self.bad_grasping_points)

        ## Filter out outdated bad_grasping_point
        self.bad_grasping_points, self.bad_grasping_times = self.remove_old_points(self.bad_grasping_points, self.bad_grasping_times, 60*3)
        if len(self.all_grasp_proposals) > 0 and not self.only_flush:

            ## Remove grasp points that are repeated:
            not_bad_grasp_points_indices = []
            for i in range(0, self.all_grasp_proposals.shape[0]):
                is_bad = False
                grasp_point = list(self.all_grasp_proposals[i, 0:self.param_grasping-1])
                for j,bad_point in enumerate(self.bad_grasping_points):
                    if len(bad_point) == self.param_grasping and  np.linalg.norm(np.array(bad_point[0:2]) - np .array(grasp_point[0:2]))< 0.03: #only looking position, not angle
                        is_bad = True
                        #e.db('..............................................................................................')
                        #e.db('Removed grasp point: ', grasp_point[0:2], ' because of previous bad point: ', bad_point[0:2])
                        break
                if not is_bad:
                    not_bad_grasp_points_indices.append(i)
            not_bad_grasp_points = np.zeros((len(not_bad_grasp_points_indices), self.param_grasping))
            not_bad_grasp_ids = ['NULL']*len(not_bad_grasp_points_indices)
            not_bad_grasp_confidence = [0]*len(not_bad_grasp_points_indices)
            for i in range(0, len(not_bad_grasp_points_indices)):
                not_bad_grasp_points[i] = self.all_grasp_proposals[not_bad_grasp_points_indices[i], :]
                not_bad_grasp_ids[i] = self.grasp_object_list[not_bad_grasp_points_indices[i]]
                not_bad_grasp_confidence[i] = self.grasp_object_confidence[not_bad_grasp_points_indices[i]]
            if len(not_bad_grasp_points) > 0:
                self.all_grasp_proposals = not_bad_grasp_points
                self.grasp_object_list = not_bad_grasp_ids
                self.grasp_object_confidence = not_bad_grasp_confidence
            print 'bad_points_grasping: ', self.bad_grasping_points
            num_points_grasp = min(num_points, len(self.all_grasp_proposals))
            self.all_pick_proposals = list(self.all_grasp_proposals[0:num_points_grasp, 0:self.param_grasping])
            self.all_pick_scores = list(self.all_grasp_proposals[0:num_points_grasp, self.param_grasping-1])
            self.all_pick_ids = list(self.grasp_object_list[0:num_points_grasp])
            self.all_pick_confidence = list(self.grasp_object_confidence[0:num_points_grasp])
        else:
            self.all_pick_proposals = []
            self.all_pick_scores = []
            self.all_pick_ids = []
            self.all_pick_confidence = []
        #Add flush proposals if possible
        if len(self.all_flush_proposals) > 0 and not self.only_grasp and container < 2:
            ## Remove flush points that are repeated:
            not_bad_flush_points_indices = []
            for i in range(0, self.all_flush_proposals.shape[0]):
                is_bad = False
                flush_point = list(self.all_flush_proposals[i, 0:self.param_flush-1])
                for j,bad_point in enumerate(self.bad_grasping_points):
                    if len(bad_point) == self.param_flush and np.linalg.norm(np.array(bad_point[0:2]) - np .array(flush_point[0:2]))< 0.025: #only looking position, not angle
                        is_bad = True
                        e.db('..............................................................................................')
                        e.db('Removed flush point: ', flush_point[0:2], ' because of previous bad point: ', bad_point[0:2])
                        break
                if not is_bad:
                    not_bad_flush_points_indices.append(i)
            not_bad_flush_points = np.zeros((len(not_bad_flush_points_indices), self.param_flush))
            not_bad_flush_ids = ['NULL']*len(not_bad_flush_points_indices)
            not_bad_flush_confidence = [0]*len(not_bad_flush_points_indices)
            for i in range(0, len(not_bad_flush_points_indices)):
                not_bad_flush_points[i] = self.all_flush_proposals[not_bad_flush_points_indices[i], :]
                not_bad_flush_ids[i] = self.flush_grasp_object_list[not_bad_flush_points_indices[i]]
                not_bad_flush_confidence[i] = self.flush_grasp_object_confidence[not_bad_flush_points_indices[i]]
            if len(not_bad_flush_points) > 0:
                self.all_flush_proposals = not_bad_flush_points
                self.flush_grasp_object_list = not_bad_flush_ids
                self.flush_grasp_object_confidence = not_bad_flush_confidence
            print 'bad_points_grasping: ', self.bad_grasping_points
            num_points_flush = min(num_points, len(self.all_flush_proposals))
            self.all_pick_proposals += list(self.all_flush_proposals[0:num_points_flush, 0:self.param_flush])
            self.all_pick_scores += list(self.all_flush_proposals[0:num_points_flush, self.param_flush-1])
            self.all_pick_ids += list(self.flush_grasp_object_list[0:num_points_flush])
            self.all_pick_confidence += list(self.flush_grasp_object_confidence[0:num_points_flush])


        '''
        pick_permutation = self.all_pick_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]
        self.grasp_object_list = self.grasp_object_list[grasp_permutation]
        '''
        self.all_pick_scores.sort(reverse=True)
        self.all_pick_proposals.sort(key=lambda x: x[-1], reverse=True)
        self.num_pick_proposals = len(self.all_pick_scores)
        self.webpage.updateValue('AreGraspPts', (self.num_pick_proposals > 0))

    ###################
    ### EXPERIMENTS ###
    ###################

    def talkWithHumanToStartExperiment(self):
        '''If runing experiments to collect data. Ask prompts and log'''
        assert 'apc_stow_task_experiment.json' in self.jsonfilename, "Probably wrong input json"
        last_input = raw_input('Do you want to use the last input?[y/n]')
        if last_input == 'y':
            last_input = raw_input('Are you sure about that?[y/n]')
            if last_input == 'y':
                self.use_last_json = True
                self.jsonfilename = os.path.join(os.environ['ARC_BASE'], 'input/last_input.json')
                self.enter_jsonfile()
                return
            else:
                e.db('I thought so')
        with open(self.jsonfilename) as data_file:
            DATA = json.load(data_file)
        #Introduce objects in tote
        jsontoteObjNames = DATA['tote']['contents']
        rand.shuffle(jsontoteObjNames)
        jsontoteObjNames = jsontoteObjNames[0:self.number_objects_experiment]
        jsontoteObjNames.sort()
        self.objects = []
        for obj in jsontoteObjNames:
            self.objects.append(Item(container=0, is_goal=self.ini_is_goal))
            self.objects[-1].update_with_label(obj)
        self.toteObj = range(len(self.objects))
        for x in range(self.num_bins):
            for obj in DATA['bins'][x]['contents']:
                self.objects.append(Item(container=x+1, is_goal = self.ini_is_goal))
                self.objects[-1].update_with_label(obj)
        self.shelfObj = range(len(DATA['tote']['contents']), len(self.objects))
        e.db('The objects that go in tote ', self.tote_ID, ' are: ', self.toteObjNames())
        see_images = raw_input("Do you want to see the images? [y/n]")
        while see_images != 'n' and see_images != 'y':
            see_images = raw_input("Do you want to see the images? [y/n]")
        see_images = (see_images == 'y')
        while see_images:
            f = pylab.figure()
            for n, obj in enumerate(self.toteObjNames()):
                f.add_subplot(3, 2, n+1) #int(number_objects_experiment/5.0) f.add_subplot(int(self.number_objects_experiment/5.0), 5, n+1)
                img = mpimg.imread('/home/mcube/Desktop/objects_images_arc_2017/%s/%s_Top_01.png' %(obj, obj))
                pylab.imshow(img)
                pylab.title(obj)
                plt.axis('off')
            pylab.show()
            see_images = raw_input("Do you want to see the images again? [y/n]")
            while see_images != 'n' and see_images != 'y':
                see_images = raw_input("Do you want to see the images again? [y/n]")
            see_images = (see_images == 'y')
        #Ask if ready
        done_with_tote = raw_input("Are you done placing the objects in the tote? [y/n]")
        while done_with_tote != 'n' and done_with_tote != 'y':
            done_with_tote = raw_input("Are you done placing the objects in the tote? [y/n]")
        done_with_tote = (done_with_tote == 'y')
        while not done_with_tote:
            done_with_tote = raw_input("Are you done placing the objects in the tote? [y/n]")
            while done_with_tote != 'n' and done_with_tote != 'y':
                done_with_tote = raw_input("Are you done placing the objects in the tote? [y/n]")
            done_with_tote = (done_with_tote == 'y')

    def init_experiment(self, args):
        '''Initializes things for experiments'''
        self.decider = 'experiment'
        self.duration_of_contest = 24*60*60 #one day
        self.previous_experiment_name = None
        self.experiment_data = {}
        self.number_objects_experiment = 5
        #Vision Logs
        self.data_directory = os.environ['ARCDATA_BASE']+'/loopdata/planner/{}'.format(int(round(time.time())))
        os.makedirs(self.data_directory)
        self.infinite_looping = True

    def json_experiment(self):
        '''Log the experimental data in json files
        @return expfilename Name of file where data is stored'''
        execution_success_string = 'success'
        if not self.execution_result:
            execution_success_string = 'fail'
        self.expfilename = os.environ['ARCDATA_BASE']+'/planner_experiments/%s_%s_%s.json' %(self.execution_date, self.best_primitive, execution_success_string)
        with open(self.expfilename, 'w') as outfile:
            json.dump(self.experiment_data, outfile, sort_keys=True,
                      indent=4, ensure_ascii=False)
        with open(self.expfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(self.experiment_data, outfile, sort_keys=True,
                      indent=4, ensure_ascii=False)
        return self.expfilename

    def GetHumanInputOnExperimentStatus(self):
        '''Prompts to ask the human about the status of the
        experiment. Queries until there is a yes or no.
        @return execution_possible (bool) whether the experiment succeeded'''
        exec_possible_input = raw_input("Did the experiment succeed? [y/n] ")
        while exec_possible_input != 'n' and exec_possible_input != 'y':
            exec_possible_input = raw_input("Did the experiment succeed? [y/n] ")
        execution_possible = (exec_possible_input == 'y')
        return execution_possible

    def GetHumanToLabelObject(self, execution_possible):
        '''Prompt to ask the human to label object. If the experiment was a
        failure, we will allow the person to report that the object is unknown.
        Otherwise the object must be known (because placing uses this
        information).
        @param execution_possible (bool) regulates if 'unknown' is valid
        @return picked_label (string) Name of the object '''
        picked_label = None
        while picked_label is None:
            for i, x in enumerate(self.toteObj):
                print '{} = {}'.format(i, self.objects[x].label)
            if not execution_possible:
                print '{} = unknown'.format(len(self.toteObj))
            which_obj = raw_input("Which object did it pick? [number] ")

            try:
                which_obj = int(which_obj)
            except ValueError:
                print 'Your input was not a number. Please input a number\n'
                continue

            if which_obj >= 0 and which_obj < len(self.toteObj):
                picked_label = self.objects[self.toteObj[which_obj]].label
            elif which_obj == len(self.toteObj) and not execution_possible:
                picked_label = 'unknown'
            else:
                print 'Not a valid number. Please input an in range number\n'
        return picked_label

    def experimentPreparation(self):
        pass
    def aux_vision_logs(self, container = None):
        '''Write to vision logs'''
        if container == None:
            container = self.tote_ID
        #################
        ## VISION LOGS ##
        #################
        self.log_file = open(self.data_directory + '/{}.txt'.format(int(round(time.time()))), 'w')
        self.log_file.write('pick source: {}\n'.format(container))
        self.log_file.write('action primitive: {}\n'.format(self.best_primitive))
        self.vision_info = self.data_directory + '/log.txt'

    def runActiveID(self, container = None, primitive_id = 'Null', primitive_id_confidence = 0):
        '''Run either real or fake vision to get the ID from active vision'''
        if container == None:
            container = self.tote_ID
        if self.visionType == 'virtual':
            self.active_vision_ID, self.active_vision_ID_small_list, self.bbox_info, self.bbox_info_bad = self.callFakeActiveVision(container)
        else:
            self.active_vision_ID, self.active_vision_ID_small_list, self.bbox_info, self.bbox_info_bad = self.CallActiveVision(container, primitive_id, primitive_id_confidence)

        self.webpage.updateValue('ActiveVisionObject', self.active_vision_ID)
        self.webpage.updateValue('ActiveVisionObjectSmall', self.active_vision_ID_small_list)
        self.bbox_info_bad = list(self.bbox_info_bad)

    def run_suction(self, container = None):
        '''Activate sution and log whether successful or not.'''
        if self.suction_point is None:
            e.db('It was suppose to do grasping, but there is no grasp proposal')
            self.webpage.updateValue('AreGraspPts', False)
            self.execution_possible = False
            self.primitive_output = None
            self.primitive_point = None
            self.primitive_score = 0
            self.suction_check = False
            return
        if container == None:
            container = self.tote_ID
        self.dirty_bin(container)
        #self.suction_point = [1.0511255264282227, -0.44232940673828125, -0.21663841009140014, 0.0, 0.0, 1.0]
        self.primitive_point = self.suction_point
        self.primitive_score = self.suction_score
        self.webpage.updateValue('SuctionScore', self.suction_score)
        if self.in_simulation:
            self.callFakeSuction(prob=0.8, container = container)
            return
        e.db(SuctionPositionTarget=self.suction_point[0:3])
        e.db(SuctionNormal=self.suction_point[3:6])
        #Prepare for suction
        if self.previous_primitive != 'suction-tote':
            goToHome.prepGripperSuction()
        flag_suction = 0
        target_list = [self.suction_point[0:3]]
        if self.suction_side:
            flag_suction = 3
        else:
            if np.linalg.norm(np.array(self.second_best_suction_point[0:2]) - np.array(self.suction_point[0:2]))> 0.15 and container < 2:
                target_list.append(self.second_best_suction_point[0:3])
        self.suction_output = suction_down_simple(listener = self.listener, br=self.br, withPause=self.withPause,
                        suction_position_target_list=target_list, flag=flag_suction, bin_id=container,viz_pub=self.viz_array_pub)

        self.primitive_output = self.suction_output
        self.plan_possible = self.suction_output['success_flag'] #suction_output['plan_possible']
        self.suction_check = self.suction_output['suction_check'] #self.suction_check = raw_input('[HELP] suck or not')=='true'
        self.go_faster_flag = self.suction_output['go_faster_flag']
        #Go home
        #suction_down_simple(listener = self.listener, br=self.br, withPause=self.withPause, flag=1, bin_id=container,viz_pub=self.viz_array_pub)
        self.execution_possible = copy.deepcopy(self.suction_check)
        e.db('Execution possible according to suction primitive: ', self.execution_possible)

    def run_grasping(self, container = None):
        '''Run either flush or normal grasping at grasp_point and log whether
        execution is successful or not
        Assumes self.grasp_point, self.grasp_score have been computed'''
        if self.grasp_point is None:
            e.db('It was suppose to do grasping, but there is no grasp proposal')
            self.webpage.updateValue('AreGraspPts', False)
            self.execution_possible = False
            self.primitive_output = None
            self.primitive_point = None
            self.primitive_score = 0
            self.suction_check = False
            return
        self.webpage.updateValue('AreGraspPts', True)
        if container == None:
            container = self.tote_ID
        self.dirty_bin(container)
        self.primitive_score = self.grasp_score
        self.primitive_point = self.grasp_point
        self.webpage.updateValue('GraspingScore', self.grasp_score)
        if self.in_simulation:
            self.callFakeGrasping(prob=0.8, container = container)
            return
        if self.previous_primitive != 'grasp-tote':
            goToHome.prepGripperPicking()
        if len(self.grasp_point) == self.param_flush:
            self.grasping_output = flush_grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                               isExecute=self.isExecute, binId=container, flag=0,
                                               withPause=self.withPause, viz_pub=self.viz_array_pub)
        else:
            self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                         isExecute=self.isExecute, binId=container, flag=0,
                                         withPause=self.withPause, viz_pub=self.viz_array_pub)
        ##Go home
        #pick(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
        #     binId=container, flag=1, withPause=self.withPause, viz_pub=self.viz_array_pub)
        #Read the primitive output
        self.pick_possible = self.grasping_output['grasp_possible']
        self.plan_possible = self.grasping_output['plan_possible']
        self.execution_possible = self.grasping_output['execution_possible']
        self.collision_detected = self.grasping_output['collision']
        self.grasping_output['graspPose'] = list(self.grasping_output['graspPose'])
        self.primitive_output = self.grasping_output
        e.db('Execution possible according to grasping primitive: ', self.execution_possible)

    def logGraspOutputForVision(self):
        '''Log the output of the grasping parameters'''
        self.log_file.write('selected_grasp_proposal: {}\n'.format(self.grasp_point))
        self.log_file.write('collision_detected: {}\n'.format(self.collision_detected))
        self.log_file.write('object_in_grasp: {}\n'.format(self.execution_possible))

    def experiment_draw(self, container = None):
        '''Execute either a suction or grasp primitive based on what vision
        has determined to be the best option '''
        if container == None:
            container = self.tote_ID
        self.best_primitive = self.primitives[0]
        self.aux_vision_logs()
        self.getBestSuctionPoint()
        self.getBestSuctionSidePoint()
        self.getBestGraspingPoint()
        if self.best_primitive == 'suction-tote':
            self.primitive_point = self.suction_point
            self.run_suction()
            self.log_file.write('selected_suction_point: {}\n'.format(self.suction_point))
            self.log_file.write('suction_success: {}\n'.format(self.suction_check))
        elif self.best_primitive == 'grasp-tote':
            self.primitive_point = self.grasp_point
            if self.grasp_point is not None:
                self.run_grasping()
                self.logGraspOutputForVision()
        else:
            assert False, 'Either suction-tote or grasp-tote'
        self.previous_primitive = copy.deepcopy(self.best_primitive)

    def planned_place(self, fixed_container = None):
        '''Call placing_planner17 based on height. Compute height map and
        either place via suction or drop in a specific location (from planner)'''
        placedObj = self.objects[self.placedObj]
        container_before_placing = copy.deepcopy(placedObj.container)
        self.pickedObjects.append(placedObj.label)
        e.db('Object that has been attempted to during placing: ', self.pickedObjects)

        self.webpage.updateValue('ObjToBePlaced', placedObj.label)

        self.PlacingPlanner.box_placing = False
        if self.task == 'picking':
            #Choose box for it
            self.choose_box_for_object(placedObj)
            if self.pickedLabel != self.active_vision_ID:  #if IDs do not match, just let it be
                placedObj.container = 1  #Put in bin 1
                e.db('The object could be a goal, but because vision results do not match, we will just put it in bin one')
            #If the object is a goal:
            if placedObj.container >= self.num_containers: #That means the object is a goal and goes into a box
                self.webpage.updateValue('IfGoal', True)
                self.PlacingPlanner.box_placing = True
                fixed_container = [placedObj.container-1]
                limit_x = 0.5*(rospy.get_param("/tote/length")-rospy.get_param('/bin%d/length'%placedObj.container))
                limit_x = max(0, int(math.floor(limit_x/self.PlacingPlanner.disc)))
                limit_y = 0.5*(rospy.get_param("/tote/width")-rospy.get_param('/bin%d/width'%placedObj.container))
                limit_y = max(0, int(math.floor(limit_y/self.PlacingPlanner.disc)))
                self.PlacingPlanner.limit_x = limit_x+3 #10
                self.PlacingPlanner.limit_y = limit_y+3 #5 #Box dependent
            elif container_before_placing == 1 and (self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0):
                self.webpage.updateValue('IfGoal', False)
                fixed_container = [0,2]
            else:
                self.webpage.updateValue('IfGoal', False)
                fixed_container = [0]  #Always place no_goal objects in bin 1 (bin 0 for planner)
        elif not self.combined_task: #In stowing only place in bin 1 (bin 0 for planner)
            fixed_container = [0]
        else:
            fixed_container = [2]
            '''
            list_objects_go_bin_2_and_3 = ['balloons', 'band_aid_tape', 'black_fashion_gloves',
                                            'hinged_ruled_index_cards', 'laugh_out_loud_jokes',
                                            'measuring_spoons', 'table_cloth', 'white_facecloth',
                                            'ticonderoga_pencils', 'colgate_toothbrush_4pk',
                                            'crayons','fiskars_scissors','irish_spring_soap',
                                            'marbles','mouse_traps', 'flashlight', 'expo_eraser',
                                            'bath_sponge','duct_tape','scotch_sponges']  #robots_everywhere, tooth, speed_stick, glue
            #
            if (self.without_active_vision or self.pickedLabel == self.active_vision_ID) and placedObj.label in list_objects_go_bin_2_and_3:
                fixed_container = [1,2]
            '''

            if placedObj.label in self.list_objects_in_bin_1:
                fixed_container = [0]


        #Update used dimensions of the object to match with bounding box from vision
        bbox_size = self.bbox_info[7:10]
        placedObj.update_w_dim_with_bbox(bbox_size)
        #Update HM
        if self.PlacingPlanner.visionType == 'real' and not self.PlacingPlanner.box_placing: #TODO: is this the best place?
            if fixed_container is not None:
                for bin_num in fixed_container:
                    self.PlacingPlanner.update_real_height_map(bin_num)
            else:
                self.PlacingPlanner.update_real_height_map()  # will update all  bins
        #Find best place object
        if fixed_container is None:
            placing_score = self.PlacingPlanner.place_object_local_best(self.placedObj) #Change container and pos placedObj,  tries 3 bins
        else:
            undesired_containers = []
            '''
            if self.combined_task and self.task == 'stowing':
                undesired_containers = [0]
                self.list_objects_in_bin_1 = ['avery_binder','composition_book', 'table_cloth', 'hanes_socks','ice_cube_tray','pie_plates']
                if placedObj.label in self.list_objects_in_bin_1:
                    undesired_containers = [2]
            '''
            placing_score = self.PlacingPlanner.place_object_local_best(self.placedObj, containers = fixed_container) #Change container and pos placedObj
            '''
            if self.combined_task:
                if self.task == 'stowing' and placing_score >= self.PlacingPlanner.INF:
                    e.db('Object does not fit, we can not put it into the same bin as the one we think is the same')
                    placing_score = self.PlacingPlanner.place_object_local_best(self.placedObj) #Put it into the big bin
            '''
        self.PlacingPlanner.update_pose_placed_obj(self.placedObj) #World pose
        self.PlacingPlanner.show_placing_decision(b = placedObj.container-1, theta = placedObj.theta , hm_pre_pub= self.hm_pre_pub, hm_post_pub= self.hm_post_pub, score_pub= self.score_pub)
        ####  TODO: Recall that objects in planner might not be de same as in pacing
        e.db('Placing ', placedObj.label ,'on bin ', placedObj.container, ' position ', placedObj.pos, 'pose' ,
                        placedObj.pose, 'given the score: ', placing_score, ' the real object dimensions are: ', placedObj.dim, ' the bbox gave dimensions: ', placedObj.w_dim)
        #Get bounding box
        drop_pose = placedObj.pose

        self.clean_bin(container_before_placing) #At this point we wont be at the container, unless placing at the same place
        self.dirty_bin(placedObj.container)

        update_command = None
        if self.task == 'picking' and container_before_placing != placedObj.container and self.visionType == 'real':
            update_command = UpdateCommand(bin_pre=container_before_placing, rm_command='{} rm {}'.format(container_before_placing, placedObj.label), funct=self.getPassiveVisionEstimate)


        #Run primitive
        if self.best_primitive == 'suction-tote':
            placing_output = suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=2,
                                bin_id=placedObj.container, suction_position_target_list = [drop_pose], obj_ID = self.pickedLabel,
                                rel_pose=self.rel_pose,BoxBody=self.BoxBody,place_pose=drop_pose,viz_pub=self.viz_array_pub,
                                is_drop=self.is_drop, update_command=update_command, go_faster_flag=self.go_faster_flag)
#            if self.PlacingPlanner.box_placing:
#                go_arc_safe(isSuction=True)
#        else:
            placing_output=grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                 isExecute=self.isExecute, objId=self.pickedLabel,
                                 binId=placedObj.container, flag=2, withPause=self.withPause,
                                 rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose,
                                 viz_pub=self.viz_array_pub, is_drop = self.is_drop, update_command=update_command)
#            if self.PlacingPlanner.box_placing:
#                go_arc_safe(isSuction=False)
        e.db('In the end, the placing was done in bin: ', placedObj.container, ' in the pose: ', placing_output['final_object_pose'][0:3], ' instead of pose: ', placedObj.pose)
        self.webpage.updateValue('PlacingLocation', placedObj.container)
        placedObj.pose = placing_output['final_object_pose'][0:3]
        if self.task == 'picking' and placedObj.container < self.num_containers:
            if self.best_primitive == 'suction-tote':
                self.bad_suction_points.append(placing_output['final_object_pose'])  #Only the three first points are meaningful
                self.bad_suction_times.append(time.time())
                condition_for_x = (placing_output['final_object_pose'][0] < self.bin1_pose[0])
                if condition_for_x and placedObj.container == 1 and not (self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0):  #condition suction point in wrong side
                    self.failed_attempts_in_high_surface +=1
            else:
                self.bad_grasping_points.append(placing_output['final_object_pose'])
                self.bad_grasping_times.append(time.time())

        if self.PlacingPlanner.box_placing:
            self.PlacingPlanner.limit_x = 6
            self.PlacingPlanner.limit_y = 2 #Manually adjusted
        self.clean_bin(placedObj.container) #No object so should be ok

    def experiment_logging(self):
        '''Create json file of the experiment'''
        self.execution_time = time.time() - self.initial_seq_time
        self.execution_date = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.validity_experiment = True
        if self.human_supervision:
            self.validity_experiment = raw_input("Is this experiment valid? [y/n]")
            while self.validity_experiment != 'n' and self.validity_experiment != 'y':
                self.validity_experiment = raw_input("Is this experiment valid? [y/n]")
            self.validity_experiment = (self.validity_experiment == 'y')
        #TODO: why execution_result = execution_possible?
        self.execution_result = self.execution_possible
        self.experiment_data = {'validity_experiment' : self.validity_experiment,
                                'execution_result' : self.execution_result,
                                'execution_date': self.execution_date,
                                'execution_time': self.execution_time,
                                'obj_ID': self.pickedLabel, #self.pickedObject.label,
                                'obj_pose': [0]*7,
                                'primitive': self.best_primitive,
                                'primitive_point': self.primitive_point,
                                'primitive_score': self.primitive_score,
                                'primitive_version': self.primitive_version,
                                'primitive_output': self.primitive_output,
                                'active_vision_ID': self.active_vision_ID,
                                'human_supervision' : self.human_supervision,
                                'num_objects': len(self.toteObj),
                                'ID_objects': self.toteObjNames(),
                                'vision_info': self.vision_info,
                                'weight_info': self.weight_info[self.tote_ID],
                                'previous_experiment_name': self.previous_experiment_name,
                                'tote_ID': self.tote_ID,
                                'bad_suction_points': self.bad_suction_points,
                                'bad_grasping_points': self.bad_grasping_points,
                                'next_experiment_name' : 'NULL'}

        self.new_experiment_name = self.json_experiment()
        e.db('This file was stored in ', self.new_experiment_name)
        #Add the 'new experiment' to the previous one
        if self.previous_experiment_name is not None:
            with open(self.previous_experiment_name) as data_file:
                data = json.load(data_file)
                data['next_experiment_name'] = self.new_experiment_name
            with open(self.previous_experiment_name, 'w') as outfile:
                json.dump(data, outfile, sort_keys=True, indent=4, ensure_ascii=False)
        self.previous_experiment_name = self.new_experiment_name
        #If human wants to stop
        if self.human_supervision:
            should_continue = raw_input("Continue with experiments? [y/n]")
            while should_continue != 'n' and should_continue != 'y':
                should_continue = raw_input("Continue with experiments? [y/n]")
            should_continue = (should_continue == 'y')
            if not should_continue:
                return True
        return False

    def switch_tote(self): #Only used in experiments
        '''Swap the ids of the shelf and tote'''
        self.tote_ID = 3-self.tote_ID
        self.toteObj, self.shelfObj = self.shelfObj, self.toteObj #swap

    ######################
    ### MAIN FUNCTIONS ###
    ######################
    def run_picking(self):
        '''Main picking function loop.'''
        if self.visionType == 'real' and not self.combined_task:
            self.picking_preparation[self.decider]()
        elif self.visionType == 'virtual':
            self.enter_jsonfile()

        if opt.interactive_filename is not '' or self.combined_task:
            if self.visionType == 'virtual' and (opt.interactive_filename is not ''):
                self.jsonfilename = self.interactive_filename
            self.enter_jsonfile()
        #raw_input('Please make sure the order file is there')

        # Send task name to reset website timer after interactive placing
        self.webpage.updateValue('TaskName', 'Picking')
        #Extend the shelf
#        shelf_helper.open_shelf(withPause=self.withPause)
        #Back to ARC position
        goToHome.goToARC(slowDown=self.goHomeSlow)

        # 1. Passive vision update bin 1-3 at the beginning of each run
        if self.visionType == 'real':
            for bin_id in xrange(1, self.num_containers):
                self.getPassiveVisionEstimate('update hm sg', '', bin_id)

        self.enter_jsonfile_boxes()
        self.enter_jsonfile_goals()
        self.webpage.updateValue('GoalLocations', self.box_goals)

        self.json_output()
        suction.pump_start()
        suction.start()
        self.num_attempts = 0
        self.num_attempts_failed = 0
        self.time_begin = time.time()
        self.time_end = self.time_begin + self.duration_of_contest

        self.all_suction_points = None
        self.container_where_robot_did_placing = 1
        self.container_tried_to_pick = None
        self.num_attempts_iterations = 0
        while True: #self.goals_left > 0: # and time.time() < self.time_end:
            self.num_attempts += 1
            e.db(GOALS_LEFT=self.goals_left)
            self.webpage.updateValue('ObjectsLeft', self.goals_left)
            self.webpage.updateValue('NumAttempts', self.num_attempts)
            self.webpage.updateValue('ObjInBins', [self.shelfObjNamesGivenBin(x+1) for x in xrange(self.num_bins)])
            self.webpage.updateValue('ObjInBoxes', [self.shelfObjNamesGivenBox(x+4) for x in xrange(5)])
            #raw_input('Ready for picking a new object?')
            self.printObjects()
            e.db('The time left is: ', self.time_end - time.time())
            if self.num_attempts_iterations >= 1:
                self.all_suction_points = None
                self.num_attempts_iterations = 0

            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)

            #Run the pick baseline
            self.pick[self.decider]()
            e.db('Primitive execution_possible result: ', self.execution_possible)
            self.webpage.updateValue('ExecutionPossible', self.execution_possible)
            #After running the primitives:
            if self.execution_possible != False:
                #Action executed in self.best_container. Check weight:
                self.weight_info[self.best_container] = self.weightSensor.readWeightSensor(item_list=self.toteObjNames(), withSensor=self.withSensorWeight, binNum=self.best_container, givenWeights= -11*int(self.execution_possible is not None))[0]

                self.webpage.updateValue('WeightDiff', self.weight_info)
                self.detected_weight = self.weight_info[self.best_container]['weights']
                self.is_drop = (self.detected_weight < 200)
                if self.detected_weight > 10:
                    self.execution_possible = True
                e.db('Detected weight:',  self.detected_weight, ' in container: ', self.best_container, ' the pick is good?(True = Yes)', self.execution_possible)
                self.webpage.updateValue('ExecutionPossible', self.execution_possible)

                if self.execution_possible == None:  #Primitive was unsure and weight was low
                    self.execution_possible = False

                #Run active vision if possible
                self.webpage.updateValue('ExecutionPossible', self.execution_possible)
                if self.execution_possible or self.forceSucc:
                    if self.best_primitive == 'grasp-tote':  #TODO this needs to be fixed by primitives
                        rospy.sleep(1)
                    e.db('Running active ID')
                    self.webpage.updateValue('ActiveVisionObject', 'Recognizing')
                    self.webpage.updateValue('ActiveVisionObjectSmall', 'Recognizing')
                    self.runActiveID(self.best_container, self.primitive_id, self.primitive_id_confidence)
                    self.pickedLabel = self.active_vision_ID_small_list

                    if self.pickedLabel == 'empty':
                        self.execution_possible = False
                    self.webpage.updateValue('ExecutionPossible', self.execution_possible)
                    e.db('----------------------------')
                    e.db('Object picked according active vision: ', self.pickedLabel)
                    e.db('----------------------------')
                    bbox_pose_ros = list(self.bbox_info[4:7] + self.bbox_info[0:4])
                    pubFrame(self.br, pose=bbox_pose_ros)
                    self.bbox_info_new = list(copy.deepcopy(self.bbox_info))
                    for i in range(7,10):
                        self.bbox_info_new[i] = max(self.bbox_info[i], self.bbox_info_bad[i])
                    self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info_new,viz_pub=self.viz_array_pub,listener=self.listener)
                    # Prevent weight being too off:
                    for obj in self.objects:
                        if obj.label == self.pickedLabel:
                            if obj.weight+0.02 < self.detected_weight/1000. and self.number_times_wrong_weight < 2:
                                e.db('The weight of the objects is 30g higher that the ID assigned by active vision')
                                self.number_times_wrong_weight += 1
                                drop_pose = get_params_yaml('bin'+str(obj.container)+'_pose')
                                update_command = None
                                if self.best_primitive == 'suction-tote':
                                    placing_output = suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=2,
                                                        bin_id=obj.container, suction_position_target_list = [drop_pose], obj_ID = self.pickedLabel,
                                                        rel_pose=self.rel_pose,BoxBody=self.BoxBody,place_pose=drop_pose,viz_pub=self.viz_array_pub,
                                                        is_drop=self.is_drop, update_command=update_command, go_faster_flag=self.go_faster_flag)
                                else:
                                    placing_output=grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                                         isExecute=self.isExecute, objId=self.pickedLabel,
                                                         binId=obj.container, flag=2, withPause=self.withPause,
                                                         rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose,
                                                         viz_pub=self.viz_array_pub, is_drop = self.is_drop, update_command=update_command)
                                self.execution_possible = False
                            break

            e.db('----------------------------')
            e.db(Execution_possible = self.execution_possible)
            e.db('----------------------------')
            self.webpage.updateValue('ExecutionPossible', self.execution_possible)

            self.container_tried_to_pick = copy.deepcopy(self.best_container)
            self.json_output()
            if (self.execution_possible or self.forceSucc): #success
                self.num_attempts_iterations = 1

                ## Check object in ShelfObj, remove it and update json
                for i, x in enumerate(self.shelfObj):
                    if self.objects[x].label == self.pickedLabel and self.objects[x].container == self.best_container:
                        self.placedObj = x
                        container_before_placing = copy.deepcopy(self.objects[x].container)
                        #Placing the object
                        self.placer[self.decider]() ########
                        goToHome.goToARC(slowDown = self.goHomeSlow)

                        obj = self.objects[x]
                        if obj.container < 4: #It was not placed in a box
                            self.num_attempts_failed += 1
                            self.container_where_robot_did_placing = obj.container

                        if self.pickedLabel != self.active_vision_ID:
                            obj.container = container_before_placing
                        #Check placing succeded using weight changes
                        if obj.container != container_before_placing:
                            #for j in range(1,9): #TODO: avoid weight failure put 8 instead 9
                            #    if j == 5 or j == 7:
                            #        self.weight_info_after_place[j] = self.weight_info_after_place[j-1]
                            #        continue
                            #    self.weight_info_after_place[j] = self.weightSensor.readWeightSensor(item_list=[], withSensor=self.withSensorWeight, binNum=j, givenWeights=obj.weight*1000)
                            self.weight_info_after_place = self.weightSensor.readWeightSensor(item_list=[], withSensor=self.withSensorWeight, binNum=range(9), givenWeights=obj.weight*1000)

                            self.webpage.updateValue('WeightDiff', self.weight_info_after_place)
                            e.db('detected weight',  self.weight_info_after_place[obj.container]['weights'], 'while the weight of the placed object is: ', obj.weight*1000, 'g')

                            if self.weight_info_after_place[obj.container]['weights'] > -5: #If object was placed there, weight given by sensors should be negative
                                e.db('It feels like the object was not placed where it was supposed to be!')
                                #Where did the object ended if not where we wanted?
                                max_weight_change = -10  #Because the object has been placed there, the weight should be negative
                                for j in range(1,9): #Look into the bins and boxes
                                    if max_weight_change > self.weight_info_after_place[j]['weights']:
                                        max_weight_change = self.weight_info_after_place[j]['weights']
                                        obj.container = j #TODO_M : distinguish between 4,5 and 6,7
                                if max_weight_change == -10:  #In that case, the object returned where it was before...
                                    obj.container = container_before_placing

                        #TODO: Move to primitives so that we wait less
                        if obj.container > 3: #Not in bins, but boxes
                            self.shelfObj.pop(i)
                            #if self.visionType == 'real':
                            #    rm_command = '{} rm {}'.format(container_before_placing, obj.label)
                            #    with Timer(rm_command):
                            #        self.getPassiveVisionEstimate('update hm state sg', rm_command, container_before_placing)   # change, picking
                            self.update_goals_after_pick(obj)
                        else:
                            if self.visionType == 'real':
                                if container_before_placing == 1 and (self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0):
                                    self.getPassiveVisionEstimate('update hm sg', '', container_before_placing)
                                    if container_before_placing != obj.container:
                                        self.getPassiveVisionEstimate('update hm', '', obj.container)
                                else:
                                    if obj.container == container_before_placing:
                                        mv_command = '{} mv {}'.format(obj.container, obj.label)
                                        with Timer(mv_command):
                                            self.getPassiveVisionEstimate('update hm sg state', mv_command, container_before_placing)   # change, picking

                                    else:
                                        #rm_command = '{} rm {}'.format(container_before_placing, obj.label)
                                        #with Timer(rm_command):
                                        #    self.getPassiveVisionEstimate('update hm state sg', rm_command, container_before_placing)   # change, picking

                                        add_command = '{} add {}'.format(obj.container, obj.label)
                                        with Timer(add_command):
                                            self.getPassiveVisionEstimate('update hm sg state', add_command, obj.container)   # change, picking

                                    self.initial_objects_bins[obj.container].append(obj.label)  #If object was moved of container, add it to the full list
                        break #We do not want to look at repeated objects
            else: #Fail
                self.num_attempts_failed += 1
                self.clean_bin(self.best_container) #No object so should be ok
                if self.best_primitive == 'suction-tote':
                    suction.stop()
                    if self.primitive_point is not None:
                        if self.with_suction_side and self.suction_side:
                            self.bad_suction_side_points.append(self.primitive_point)
                            self.bad_suction_side_times.append(time.time())
                        else:
                            self.bad_suction_points.append(self.primitive_point)
                            self.bad_suction_times.append(time.time())
                            condition_for_x = (self.primitive_point[0] < self.bin1_pose[0])
                            if False and self.best_container == 1 and not (self.num_attempts_failed > 4 or self.numGoalObjectsContainer(1) == 0):  #condition suction point in wrong side
                                self.failed_attempts_in_high_surface += 1
                else:
                    if self.primitive_point is not None:
                        self.bad_grasping_points.append(self.primitive_point)
                        self.bad_grasping_times.append(time.time())
                if self.visionType == 'real':
                    #SS: not call 'mv' without active vision
                    self.getPassiveVisionEstimate('update hm sg', '', self.best_container)   # change, picking




                self.num_attempts_iterations += 1  #If lower than 1 we should try to pick without doing vision
                #Prevent problems with active vision:
                goToHome.goToARC(slowDown = self.goHomeSlow)

            self.json_output()

        #Finishing picking
        goToHome.goToARC(slowDown = self.goHomeSlow)
        e.db("Planner is done")

    def run_stowing(self):
        '''Main stowing function loop. This will loop until all objects are cleared
        or the time limit is hit. Generically calls each action, with the mode
        being indicated by the self.decider. Stows objects and logs all data'''

        self.preparation[self.decider]()

        if self.combined_task:
            self.picking_preparation[self.decider]()

        #JSON
        self.enter_jsonfile()

        self.time_begin = time.time()
        self.time_end = self.time_begin + self.duration_of_contest
        # 0. Extend the shelf
        if self.combined_task:
            shelf_helper.open_shelf(withPause=self.withPause)
        # 1. Initialize robot state
        goToHome.goToARC(slowDown=self.goHomeSlow)
        self.webpage.updateValue('RobotAction', 'Preparation')

        self.json_output()
        self.webpage.updateValue('TaskName', 'Stowing') #resets time
        self.json_output()
        suction.pump_start()
        suction.start()

        # 2. Passive vision update bin 0-3 at the beginning of each run
        if self.visionType == 'real':
            e.db("getPassiveVisionEstimate 'update hm sg', '', ", self.tote_ID)
            self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
            if not self.combined_task:
                n_hm_cam = 2
            else:
                n_hm_cam = 4
            for bin_id in range(1,n_hm_cam):
                e.db("getPassiveVisionEstimate 'update hm', '', ", bin_id)
                self.getPassiveVisionEstimate('update hm', '', bin_id)

        # 3. Start the stowing loop
        self.all_suction_points = None
        self.num_attempts_iterations = 0
        self.num_attempts = 0
        self.num_attempts_failed = 0

        #while len(self.toteObj) > 0 and time.time() < self.time_end:
        while True:
            #raw_input('Next try')
            self.initial_seq_time = time.time()
            self.num_attempts += 1
            e.db('The time left is: ', self.time_end - self.initial_seq_time)
            e.db(num_attempts_iterations = self.num_attempts_iterations)
            self.webpage.updateValue('ObjectsLeft', len(self.toteObj))
            self.webpage.updateValue('NumAttempts', self.num_attempts)
            self.webpage.updateValue('ObjInTote', self.toteObjNames())
            self.webpage.updateValue('ObjInBins', [self.shelfObjNamesGivenBin(x+1) for x in xrange(self.num_bins)])
            if self.num_attempts_iterations >= 1: #By now we always want to call vision
                self.all_suction_points = None
                self.num_attempts_iterations = 0

            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)

            #Execute best primitive
            self.draw[self.decider]()
            self.webpage.updateValue('ExecutionPossible', self.execution_possible)
            if self.execution_possible != False: #Primitive reasoning
                #Use weight to check
                self.webpage.updateValue('RobotAction', 'Weight Check')
                self.weight_info[self.tote_ID] = self.weightSensor.readWeightSensor(item_list = self.toteObjNames(), withSensor=self.withSensorWeight, binNum=self.tote_ID, givenWeights=-11*int(self.execution_possible is not None))[0]

                self.webpage.updateValue('WeightDiff', self.weight_info)
                if self.weight_info[self.tote_ID]['weights'] > 10:
                    self.execution_possible = True
                max_prob_index = (self.weight_info[self.tote_ID]['probs']).tolist().index(max(self.weight_info[self.tote_ID]['probs']))
                if max_prob_index == len(self.weight_info[self.tote_ID]['probs'])-1:
                    self.execution_possible = False
                    if len(self.toteObjNames()) == 0 and self.weight_info[self.tote_ID]['weights'] > 10:
                        self.execution_possible = True
                        max_dim = 0
                        for obj in self.shelfObj:
                            if self.objects[obj].dim[0] > max_dim:
                                self.pickedLabel = copy.deepcopy(self.objects[obj].label)
                                max_dim = self.objects[obj].dim[0]
                    else:
                        if self.weight_info[self.tote_ID]['weights'] > 10:
                            list_without_last = (self.weight_info[self.tote_ID]['probs']).tolist()
                            list_without_last = list_without_last[:-1]
                            max_prob_index = list_without_last.index(max(list_without_last))
                            self.execution_possible = True
                            toteObjects = self.toteObjNames()
                            self.pickedLabel = toteObjects[max_prob_index]
                else:
                    toteObjects = self.toteObjNames()
                    self.pickedLabel = toteObjects[max_prob_index]
                e.db('Detected weight:',  self.weight_info[self.tote_ID]['weights'], ' execution seems possible?(True = Yes)', self.execution_possible)
                self.webpage.updateValue('ExecutionPossible', self.execution_possible)

                if self.execution_possible == None:  #Primitive was unsure and weight was low
                    self.execution_possible = False
                self.webpage.updateValue('ExecutionPossible', self.execution_possible)

                # Query the user to determine if the experiment succeeded
                if self.human_supervision:
                    self.execution_possible = self.GetHumanInputOnExperimentStatus()
                self.webpage.updateValue('ExecutionPossible', self.execution_possible)

                if self.execution_possible or self.forceSucc:
                    e.db('Running active ID')
                    self.webpage.updateValue('RobotAction', 'Running Active ID')
                    ## Move to bin 1 for active vision
                    self.webpage.updateValue('RobotAction', self.best_primitive)
                    if self.best_primitive == 'grasp-tote':
                        grasp(objInput=[], listener=self.listener, br=self.br, isExecute=self.isExecute, objId = self.pickedLabel,
                              binId=1, flag=1, withPause=self.withPause, viz_pub=self.viz_array_pub)
                    else:
                        suction_down_simple(listener = self.listener, br=self.br, withPause=self.withPause, obj_ID = self.pickedLabel , flag=1, bin_id=1,viz_pub=self.viz_array_pub,go_faster_flag=self.go_faster_flag)

                    #Update the weight value after moving #TODO, happy about this?
                    self.weight_info[self.tote_ID] = self.weightSensor.readWeightSensor(item_list = self.toteObjNames(), withSensor=self.withSensorWeight, binNum=self.tote_ID, givenWeights=-11*int(self.execution_possible is not None))[0]
                    self.detected_weight = self.weight_info[self.tote_ID]['weights']
                    self.is_drop = (self.detected_weight < 200)
                    self.webpage.updateValue('WeightDiff', self.weight_info)
                    e.db('Detected weight after going to active vision position in bin 1:',  self.detected_weight)

                    # Right after moving to active vision position, update bin 0 grasp/suction
                    if self.visionType == 'real':
                        self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)

                    if not self.without_active_vision:
                        self.webpage.updateValue('RobotAction', self.best_primitive)
                        if self.best_primitive == 'grasp-tote':  #TODO this needs to be fixed by primitives
                            rospy.sleep(1)
                        self.webpage.updateValue('RobotAction', 'Running Active Vision')
                        self.webpage.updateValue('ActiveVisionObject', 'Recognizing')
                        self.webpage.updateValue('ActiveVisionObjectSmall', 'Recognizing')
                        self.runActiveID(self.tote_ID, 'Null', 0)
                        self.pickedLabel = self.active_vision_ID_small_list
                    else:
                        max_prob_index = (self.weight_info[self.tote_ID]['probs']).tolist().index(max(self.weight_info[self.tote_ID]['probs']))
                        if max_prob_index == len(self.weight_info[self.tote_ID]['probs'])-1:
                            self.pickedLabel = 'empty'
                        else:
                            toteObjects = self.toteObjNames()
                            self.pickedLabel = toteObjects[max_prob_index]
                            max_dimension = 0.1
                            for obj in self.objects:
                                if obj.label == self.pickedLabel:
                                    max_dimension = max(obj.dim)
                                    break
                            self.bbox_info = fake_bbox_info_1(self.listener)
                            self.bbox_info[7:10] = [max_dimension, max_dimension, max_dimension]
                            self.bbox_info_bad = copy.deepcopy(self.bbox_info)
                            self.active_vision_ID = copy.deepcopy(self.pickedLabel)
                    fixed_container = None
                    if self.pickedLabel == 'empty':
                        self.execution_possible = False
                        if self.best_primitive == 'grasp-tote':  #In this case, there might be something in the gripper, just take it into account and open it
                            gripper.open(400)
                    elif not self.without_active_vision and self.pickedLabel != self.active_vision_ID:  #We might be facing a mistake, we want to place them in the same bin
                        e.db( 'Active vision IDs do not match. Small list: ', self.pickedLabel, 'Big list: ', self.active_vision_ID)
                        for obj in self.objects:
                            if obj.label == self.active_vision_ID:
                                fixed_container = [copy.deepcopy(obj.container)-1]
                        e.db( 'The object should be place in the container: ', fixed_container[0]+1, ' because we are unsure about its ID')
                    
                    if self.execution_possible:
                        e.db('Object picked: ', self.pickedLabel, ' and the bbox_info is: ',self.bbox_info)
                        bbox_pose_ros = list(self.bbox_info[4:7] + self.bbox_info[0:4])  # change matlab quat to our convention [x,y,z, qx,qy,qz,qw]
                        pubFrame(self.br, pose=bbox_pose_ros)
                        self.bbox_info_new = list(copy.deepcopy(self.bbox_info))
                        for i in range(7,10):
                            self.bbox_info_new[i] = max(self.bbox_info[i], self.bbox_info_bad[i])
                        self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info_new,viz_pub=self.viz_array_pub,listener=self.listener)
                        # Prevent weight being too off:
                        for obj in self.objects:
                            if obj.label == self.pickedLabel:
                                if obj.weight+0.02 < self.detected_weight/1000. and self.number_times_wrong_weight < 2:
                                    e.db('The weight of the objects is 30g higher that the ID assigned by active vision')
                                    self.number_times_wrong_weight += 1
                                    drop_pose = get_params_yaml('bin'+str(obj.container)+'_pose')
                                    update_command = None
                                    if self.best_primitive == 'suction-tote':
                                        placing_output = suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=2,
                                                            bin_id=obj.container, suction_position_target_list = [drop_pose], obj_ID = self.pickedLabel,
                                                            rel_pose=self.rel_pose,BoxBody=self.BoxBody,place_pose=drop_pose,viz_pub=self.viz_array_pub,
                                                            is_drop=self.is_drop, update_command=update_command, go_faster_flag=self.go_faster_flag)
                                    else:
                                        placing_output=grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                                             isExecute=self.isExecute, objId=self.pickedLabel,
                                                             binId=obj.container, flag=2, withPause=self.withPause,
                                                             rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose,
                                                             viz_pub=self.viz_array_pub, is_drop = self.is_drop, update_command=update_command)
                                    self.execution_possible = False
                                break
                else:
                    # For case not going through active vision, update bin 0 grasp/suction
                    if self.visionType == 'real':
                        self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)

                if self.human_supervision:
                    e.db('ID given by active vision: ', self.active_vision_ID)
                    self.pickedLabel = self.GetHumanToLabelObject(self.execution_possible)

                if self.experiment:
                    self.log_file.write('pick_successful: {}\n\n\n'.format(self.execution_possible))
            else:
                # For case not going through active vision, update bin 0 grasp/suction
                if self.visionType == 'real':
                    self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)

            self.json_output()
            e.db('----------------------------------------------')
            e.db(Execution_possible = self.execution_possible)
            e.db('----------------------------------------------')
            if (self.execution_possible or self.forceSucc): #success
                self.fails_in_row = 0
                self.num_attempts_iterations = 1
                ##Place object and check final position
                self.webpage.updateValue('RobotAction', 'Place Object')
                object_found = False
                for i, x in enumerate(self.toteObj):
                    if self.objects[x].label == self.pickedLabel:
                        object_found = True
                        self.placedObj = x
                        container_before_placing = copy.deepcopy(self.objects[x].container)
                        #Running the place
                        self.placer[self.decider](fixed_container)
                        obj = self.objects[x]
                        #Update weight sensors outputs
                        number_bins_used = 4
                        if not self.combined_task:
                            number_bins_used = 2
                        #for j in range(number_bins_used):
                        #    self.weight_info_after_place[j] = self.weightSensor.readWeightSensor(item_list=[], withSensor=self.withSensorWeight, binNum=j, givenWeights=obj.weight*1000)
                        self.weight_info_after_place = self.weightSensor.readWeightSensor(item_list=[], withSensor=self.withSensorWeight, binNum=range(number_bins_used), givenWeights=obj.weight*1000)
                        self.webpage.updateValue('WeightDiff', self.weight_info_after_place)
                        e.db('detected weight',  self.weight_info_after_place[obj.container]['weights'], 'while the weight of the placed object is: ', obj.weight*1000)

                        if self.weight_info_after_place[obj.container]['weights'] > -5:  #Virtual mode never enters here...
                            e.db('It feels like the object was not placed where it was supposed to be!')
                            #The object was not placed as expected
                            max_weight_change = -10  #Because the object has been placed there, the weight should be negative
                            for j in range(number_bins_used): #Look into the bins and boxes
                                if max_weight_change > self.weight_info_after_place[j]['weights']:
                                    max_weight_change = self.weight_info_after_place[j]['weights']
                                    obj.container = j
                            if max_weight_change == -10:  #In that case, the object remained where it was before
                                obj.container = container_before_placing

                        if obj.container != self.tote_ID:
                            self.shelfObj.append(x)
                            self.toteObj.pop(i)
                            if self.visionType == 'real':
                                if self.combined_task:
                                    add_command = '{} add {} {} {} {}'.format(obj.container,obj.label, obj.pose[0], obj.pose[1], obj.pose[2])
                                    with Timer('Call passive vision to update state: ' + add_command + '%d' % obj.container):
                                        e.db('getPassiveVisionEstimate', 'update hm sg state', add_command, obj.container)
                                        self.getPassiveVisionEstimate('update hm sg state', add_command, obj.container)  # change, in stowing
                                else:
                                    self.getPassiveVisionEstimate('update hm', '', obj.container)
                        else: # Case where the object ended by mistake in the tote
                            if self.visionType == 'real':
                                self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
                        break #We do not want to look at repeated objects
                if not object_found:
                    if self.best_primitive == 'grasp-tote':  #In this case, there might be something in the gripper, just take it into account and open it
                        gripper.open(400)
                    else:
                        suction.stop()
            else: #Fail
                self.num_attempts_failed += 1
                self.clean_bin(self.tote_ID)
                if self.best_primitive == 'suction-tote':
                    suction.stop()
                    #suction.pump_stop()
                    if self.primitive_point is not None:
                        if self.with_suction_side and self.suction_side:
                            self.bad_suction_side_points.append(self.primitive_point)
                            self.bad_suction_side_times.append(time.time())
                        else:
                            self.bad_suction_points.append(self.primitive_point)
                            self.bad_suction_times.append(time.time())
                else:
                    if self.primitive_point is not None:
                        self.bad_grasping_points.append(self.primitive_point)
                        self.bad_grasping_times.append(time.time())
                self.fails_in_row += 1
                self.num_attempts_iterations += 1
            self.json_output()
            stop_experiment = self.store_info[self.decider]()
            if stop_experiment:
                break

            if self.decider == 'experiment' and self.fails_in_row > 9:
                if self.infinite_looping:
                    e.db('The pick failed 10 times in a row, switching totes')
                    self.switch_tote()
                else:
                    e.db('The pick failed 10 times in a row, stopping')
                    break

            if self.decider == 'experiment' and len(self.toteObj) == 0 and self.infinite_looping:
                self.switch_tote()

        # Finished stowing
        goToHome.goToARC(slowDown = self.goHomeSlow)
        e.db("Planner is done")
        self.webpage.updateValue('RobotAction', 'Planner is Done')

    def run(self):
        #Run the right task
        '''Exeecutes only the stowing task'''
        if self.task == 'stowing':
            self.webpage.updateValue('TaskName', 'Stowing')
            self.run_stowing()
        elif self.task == 'picking':
            self.webpage.updateValue('TaskName', 'Picking')
            self.run_picking()
        else:
            assert False, 'I can only do stowing or picking'

if __name__ == '__main__':
    ##
    rospy.init_node('Planner') #, log_level=rospy.DEBUG)
    ## Explainer for outputs
    e = Explainer()
    e.set_current_theme('planner')
    ## Parse arguments
    parser = optparse.OptionParser()

    parser.add_option('-v', '--vision', action='store', dest='visionType',
        help='real or virtual', default='real')

    parser.add_option('--pvfile', action='store', dest='passive_vision_file_id',
        help='Path of the passive vision file to use', default='full_bin')

    parser.add_option('-p', '--pause', action='store_true', dest='withPause',
        help='To pause or not', default=False)

    parser.add_option('-n', '--noexe', action='store_false', dest='isExecute',
        help='To execute or not', default=True)

    parser.add_option('-f', '--forcesucc', action='store_true', dest='forceSucc',
        help='To force success on trying the last strategy in the strategy list; This is useful for testing system in virtual environment.',
        default=False)

    parser.add_option('-j', '--jsonfilename', action='store', dest='jsonfilename',
        help='Name of the json file', default='apc_stow_task17.json')

    parser.add_option('-o', '--jsonorder', action='store', dest='json_order_filename',
        help='Name of the json file containing the order', default='')

    parser.add_option('-a', '--actions', action='store', dest='primitives',
        help='Subset of suction-tote, grasp-tote or all', default='all')

    parser.add_option('-t', '--themes', action='store', dest='themes',
        help='Subset of hardware, planner, manipulation, debug or all', default='all')

    parser.add_option('-l', '--levels', action='store', dest='levels',
        help='Value from 0 (silence) to 5 (all)', default='5')

    parser.add_option('-s', '--simulation', action='store_true', dest='in_simulation',
        help='Whether to run the simulated primitives', default=False)

    parser.add_option('-e', '--experiment', action='store_true', dest='experiment',
        help='Whether to run passive vision experiments', default=False)

    parser.add_option('-m', '--human_supervision', action='store_true', dest='human_supervision',
        help='Whether human answers questions each pick', default=False)

    parser.add_option('-d', '--decider', action='store', dest='decider',
        help='What decider to user (experiment[default], baseline, random)', default='experiment')

    parser.add_option('--duration', action='store', dest='duration_of_contest',
        help='Duration of the contest in seconds', default=15*60)

    parser.add_option('--num_pick', action='store', dest='num_pick',
        help='Number of objects considered during picking', default=32)

    parser.add_option('--num_combined_stow', action='store', dest='num_combined_stow',
        help='Number of objects considered during the stowing part of the combined task', default=16)

    parser.add_option('--task', action='store', default='stowing',
        help='Task to run [stowing[def]/picking]')

    parser.add_option('--combined', action='store_true', dest='combined_task',
        help='Whether we are running the combined task or not', default=False)

    parser.add_option('--without_active_vision', action='store_true', dest='without_active_vision',
        help='Whether we are running the combined task or not', default=False)

    parser.add_option('--interactive', action='store', dest='interactive_filename',
        help='Name of the json file containing the information of the interactive placing', default='')

    (opt, args) = parser.parse_args()

    #Explainer
    for theme in e.filtered_theme:
        if (theme in opt.themes) or (opt.themes == 'all'):
            e.filtered_theme[theme] = False
        else:
            e.filtered_theme[theme] = True
    e.set_verbosity(int(opt.levels))
    #sys.stdout = Logger() #Sure?

    (opt, args) = parser.parse_args()

    #Creating necessary directories
    if not os.path.isdir(os.environ['ARCDATA_BASE']+'/loopdata/planner/'):
        os.makedirs(os.environ['ARCDATA_BASE']+'/loopdata/planner')
    if not os.path.isdir(os.environ['ARCDATA_BASE']+'/planner_experiments'):
        os.makedirs(os.environ['ARCDATA_BASE']+'/planner_experiments')
    if not os.path.isdir(os.environ['ARC_BASE']+'/output'):
        os.makedirs(os.environ['ARC_BASE']+'/output')

    p = TaskPlanner(args)
    p.run()
