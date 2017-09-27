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
#from ik.ik import *
from ik.helper import get_params_yaml, vision_transform_precise_placing_with_visualization
import numpy as np
import tf.transformations as tfm
from manual_fit.srv import *
import os
from collision_detection.collisionHelper import collisionCheck
import suction
from apc.helper import loadHeuristic, loadHeuristic_2016, sortOrder, displayOrder, displayOrder_placing_2016, bin_id2num
from sensor_msgs.msg import Image as RosImage
import shelf_helper
import webpages
from collision_free_placing import collision_free_placing

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
from ik.helper import get_obj_vol, rotmatZ, get_obj_dim, matrix_from_xyzquat, pauseFunc, getObjCOM, quat_from_matrix, in_or_out, moveGripper
from ik.marker_helper import createArrowMarker, createCubeMarker2, createDeleteAllMarker
from visualization_msgs.msg import MarkerArray, Marker
import spatula
from explainer import Explainer

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
        self.param_grasping = 12
        self.param_flush = 6
        self.penalty_bad_suction = 1.
        self.penalty_bad_grasping = 1.
        self.passive_vision_file_path = 'full_bin'
        ##################################
        ### Assertions before starting ###
        ##################################
        assert ((not opt.forceSucc) or opt.in_simulation), 'Can only force success in simulation'
        assert opt.visionType in self.passiveVisionTypes, 'visionType should be in'+str([_ for _ in self.passiveVisionTypes])+'but its '+opt.visionType
        assert (opt.visionType == 'real' or (not opt.experiment)), 'visionType should be real during experiments'
        assert (opt.in_simulation or opt.visionType != 'virtual'), 'we should be in simulation to use virtual vision'
        e.db(visionType = opt.visionType)
        e.db(in_simulation = opt.in_simulation)
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
        assert os.path.isfile(self.jsonfilename), 'JSON file leads nowhere'
        #####################
        ### Configuration ###
        #####################
        self.in_simulation = opt.in_simulation
        self.withPause = opt.withPause
        self.forceSucc = opt.forceSucc
        self.duration_of_contest = opt.duration_of_contest
        self.experiment = opt.experiment
        self.task = opt.task
        self.isExecute = opt.isExecute
        self.human_supervision = opt.human_supervision
        self.combined_task = False
        
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
        self.goals = []
        self.goals_left = 0 #number of objects to be placed in boxes
        self.objects = [] #List of objects in this run [in all places]
        self.toteObj = [] #List of indices to the object list of objects in tote
        self.shelfObj = [] #List of indices to the object list of objects in shelf
        self.box_id = [] 
        self.box_size = []
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
        ## Manipulation
        self.goHomeSlow = False
        self.primitive_version = 1
        self.dirty_bins = ["0"]*4 #All bins start clean
        self.placedObj = None
        self.time_since_clean = [math.floor(time.time())]*4
        
        ## ROS setup
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #self.publisher = rospy.Publisher('dirty_bins', std_msgs.msg.String, queue_size=10)
        
        if self.visionType == 'real':
            self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
            for i in range(1, self.num_containers):
                restart_command = '{} restart -'.format(i)
                self.getPassiveVisionEstimate(restart_command, i)
        

        #JSON
        self.use_last_json = False
        self.enter_jsonfile()

        #Bad points
        self.bad_suction_points = [] #list of failed attempts
        self.bad_grasping_points = []
        self.bad_suction_times = [] #list of times when we failed
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
        ########################################
        ### Potentially not useful variables ###
        ########################################
        self.weight_info = [None for _ in range(9)]  #Adding the boxes
        self.weight_info_after_place = [None for _ in range(9)] #Adding the boxes 
        self.vision_info = None
        self.active_vision_ID = None
        e.db(decider=self.decider)
        e.db(experiment=self.experiment)

        ########################################
        ###         Class Publishers         ###
        ########################################
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.hm_pre_pub = rospy.Publisher('/height_map_pre', RosImage, queue_size = 10)
        self.hm_post_pub = rospy.Publisher('/height_map_post', RosImage, queue_size = 10)
        self.score_pub = rospy.Publisher('/placing_score', RosImage, queue_size = 10)
        rospy.sleep(0.5)


    ###############################
    ### GLOBAL HELPER FUNCTIONS ###
    ###############################
    def get_estimate(self, command = "", bin_id = None):
        if self.visionType == 'real':
            self.getPassiveVisionEstimate(command, bin_id)

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
            self.toteObj = range(len(self.objects))
            ##Introduce objects in shelfs
            for x in range(self.num_bins):
                for obj in DATA['bins'][x]['contents']: 
                    self.objects.append(Item(container=x+1,is_goal=self.ini_is_goal)) 
                    self.objects[-1].update_with_label(obj)
                    #TODO_M: these needs to come from the previous placing of the picking objects

            self.shelfObj = range(len(DATA['tote']['contents']), len(self.objects))
            if self.use_last_json:
                e.db(toteObjNames=self.toteObjNames)
                finished_placing = 'n'
                while finished_placing != 'y':
                    finished_placing = raw_input('Have you finished placing the objects?[y/n]')
            ##Introduce boxes
            if self.json_boxes_filename != os.environ['ARC_BASE'] + '/input/':  #No file name assigned to it
                with open(self.json_boxes_filename) as data_file:
                    DATA_JSON = json.load(data_file)
                self.num_boxes = 0
                for box_num, box in enumerate(DATA_JSON['boxes']):
                    self.box_id.append(box['size_id'])
                    self.box_size.append(box['dimensions'])
                    self.num_boxes += 1
            
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
                            self.box_goals[box_num] = copy.copy(box_goal['contents'])
                            for goal_label in box_goal['contents']:
                                for obj in self.objects:
                                    if obj.label == goal_label:
                                        obj.is_goal = 'yes'
                                        self.goals.append(obj.label)
            if self.task == 'picking':
                #In picking, we know whether you're a goal or not; there's no maybe
                for obj in self.objects:
                    #Convert 'maybe' in 'yes' or 'no'
                    if obj.is_goal != 'yes':
                        obj.is_goal = 'no'

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
        bbox_info = [1,0,0,0,0.9949660234451294, -0.5, .2,0,0,0,0]
        for i, obj in enumerate(self.objects):
            if obj.label == self.pickedLabel and obj.container == container:
                bbox_info[7:10] = obj.dim
        return self.pickedLabel, bbox_info

    def callFakePassiveVision(self, container):
        '''For Fake Passive vision, read fake results of vision from a
        data file and pull out the suction and grasp proposals. Param container is unused'''
        e.db('File used for fake passive vision: ', self.passive_vision_file_id)
        full_path = self.FAKE_PASSIVE_VISION_DIR + self.passive_vision_file_id
        with open(full_path + '/passive_vision_data_bin_' + '0' + '.json', 'r') as infile: #TODO_M : modified to avoid problems with files
            DATA = json.load(infile)
        self.all_suction_points = [float(i) for i in DATA['suction_points']]
        self.all_suction_points = np.array(self.all_suction_points)
        self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/7, 7)
        self.all_suction_points = self.all_suction_points[self.all_suction_points[:, 7-1].argsort()[::-1]]

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
        #Run the primitive in virtual
        if len(self.grasp_point) == self.param_flush or self.only_flush:
            e.db('------- DOING FLUSH ------- ')
            e.db( grasp_point = self.grasp_point)
            _ = flush_grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                            binId=container, flag=0, withPause=self.withPause)
        else:
            e.db('------- DOING GRASPING ------- ')
            e.db( grasp_point = self.grasp_point)
            _ = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                      binId=container, flag=0, withPause=False)
        #Go to active vision position
        grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute, binId=container, flag=1, withPause=self.withPause)
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
        #Just run it for visualization purposes:
        suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause,
                        suction_position_target_list=[self.suction_point[0:3]],
                        surface_normal_list=[self.suction_point[3:6]], flag=0,
                        bin_id=container,
                        viz_pub=self.viz_array_pub)
        #Go to active vision position
        suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=1, bin_id=container, viz_pub=self.viz_array_pub)
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
        e.db('No a goal object, so it is placed in a bin')

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
                break #we only want to remove one goal
        #TODO_M: update is_goal params in case same label no longer needed

    def baseline_pick(self):
        '''This function is in charge of deciding what to pick next'''
        #Get IDs of the containers [most likely 1,2,3]
        self.container_IDs = []
        for i in range(self.num_containers):
            if i != self.tote_ID:
                self.container_IDs.append(i)
        #Get the best suction and grasping points for every container
        self.best_points = []
        for container in self.container_IDs:
            if self.numGoalObjectsContainer(container) == 0:
                continue
            if 'suction-tote' in self.primitives:
                self.getBestSuctionPoint(container)
                self.best_points.append({'score' : self.suction_score,
                                                 'point': self.suction_point,
                                                 'container': container,
                                                 'primitive': 'suction-tote'})
            if 'grasp-tote' in self.primitives:
                self.getBestGraspingPoint(container)
                self.best_points.append({'score' : self.grasp_score,
                                                 'point': self.grasp_point,
                                                 'container': container,
                                                 'primitive': 'grasp-tote'})
            self.all_suction_points = None
        if len(self.best_points) == 0:
            e.db('No best point has been obtained, try to call baseline_pick again')
            self.pick[self.decider]()
            return

        #Sort the best points according to their score and select the best one
        self.best_points.sort(key = lambda x : x['score'], reverse = True)
        
        self.primitive_point = self.best_points[0]['point']
        self.primitive_score = self.best_points[0]['score']
        self.best_container = self.best_points[0]['container']
        self.best_primitive = self.best_points[0]['primitive']
        print self.best_points
        raw_input('Best points')
        #Execute the best primitive
        if self.best_primitive == 'suction-tote':
            self.best_primitive = 'suction-tote'
            e.db('-----------------------------')
            e.db('RUNNING SUCTION IN CONTAINER ', self.best_container)
            e.db('-----------------------------')
            self.suction_point = copy.deepcopy(self.primitive_point)
            self.suction_score = copy.deepcopy(self.primitive_score)
            e.db( 'The suction point is: ', self.suction_point, ' and has an score of: ', self.suction_score)
            self.run_suction(container=self.best_container)
        else:
            self.best_primitive == 'grasp-tote'
            e.db('-----------------------------')
            e.db('RUNNING GRASPING IN CONTAINER', self.best_container)
            e.db('-----------------------------')
            self.grasp_point = copy.deepcopy(self.primitive_point)
            self.grasp_score = copy.deepcopy(self.primitive_score)
            e.db( 'The grasping point is: ', self.grasp_point, ' and has an score of: ', self.grasp_score)
            self.run_grasping(container=self.best_container)

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

    def pickingProposalsFilter(self):
        
        if len(self.all_suction_points) > 0:
            for i in range(0, self.all_suction_points.shape[0]):
                if self.suction_object_list[i] not in self.goals:
                    self.all_suction_points[i, self.param_suction-1] *= 0.1 #You do not want to consider this point if it has a different ID
                if self.suction_object_list[i] == 'NULL':
                    pass
                #TODO_M: we could also consider what is the object that it is closest to it
                #TODO_M: we could also consider its visibility and objects on top
                    
                
        if len(self.all_grasp_proposals) > 0:    
            for i in range(0, self.all_grasp_proposals.shape[0]):
                if self.grasp_object_list[i] not in self.goals:
                    self.all_grasp_proposals[i, self.param_grasping-1] *= 0.1 #You do not want to consider this point if it has a different ID
                if self.grasp_object_list[i] == 'NULL':
                    pass
        if len(self.all_flush_proposals) > 0:        
            for i in range(0, self.all_flush_proposals.shape[0]):
                if self.flush_grasp_object_list[i] not in self.goals:
                    self.all_flush_proposals[i, self.param_flush-1] *= 0.1 #You do not want to consider this point if it has a different ID
                if self.flush_grasp_object_list[i] == 'NULL':
                    pass
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
    def pickingPreparation(self):
        
        if opt.interactive_filename is not '':
            self.read_interactive_file()
            return
        DATA = {}
        DATA['label'] = []; DATA['container'] = []; DATA['w_dim'] = []; DATA['pose'] = []; DATA['theta'] = []; DATA['pos'] = [];
        self.placing_index_in_container = [];  #SS: store the ordering of placing
        #SS: get initial HM
        if self.PlacingPlanner.visionType == 'real': 
            # SS: enable passive vision
            self.getPassiveVisionEstimate('dirty 0000', 0)
            self.PlacingPlanner.update_real_height_map()   
            # SS: disable passive vision
            self.getPassiveVisionEstimate('dirty 1111', 0)
            print('[ PLANNER ] Waiting 10 seconds for passive vision to pause.')      
            time.sleep(10) # SS: wait for passive vision to stop updating

        e.db('[ PLANNER ] Ready for placing objects.')      
        for j in range(len(self.objects)):
            #now only go through the list
            print('Name of object #'+str(j)+': '+self.objects[j].label)    
            input_object_id = j
            
            obj = self.objects[input_object_id]
            # suggeste best bin
            best_container = self.PlacingPlanner.place_object_choosebin(input_object_id)
            e.db(obj.label, 'go in bin', obj.container)

            '''
            #Get best z position
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
                #Find best place object given dimension changed
                score_for_bin = self.PlacingPlanner.place_object_local_best(input_object_id) #Change container and pos placedObj
                if best_score > int(score_for_bin):
                    best_score = score_for_bin
                    best_z = z
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
            e.db(obj.label, 'go in bin', obj.container, 'in the pose', obj.pose, 'or', obj.pos, 'in the tote frame and theta', obj.theta)
            '''
            


            #SS: reset the bin id use user input only do it once directly reset it
            if self.visionType == 'real':
                do_you_like = raw_input('Is it good?[y/n]')
                while do_you_like != 'n' and do_you_like != 'y':
                    do_you_like = raw_input('Is it good?[y/n]')
                do_you_like = (do_you_like == 'y')
                if not do_you_like:
                    bin_num = raw_input('What bin is the object placed? [1/2/3]')
                    while bin_num != '1' and bin_num != '2' and bin_num != '3':
                          bin_num = raw_input('What bin is the object placed? [1/2/3]')
                    bin_num = int(bin_num)
                    obj.container = bin_num
                    e.db(obj.label, 'go in bin', obj.container, 'with unknown pose')

            #Update its position and add to the map
            self.PlacingPlanner.update_pose_placed_obj(input_object_id)

            #Show result of placing:
            self.PlacingPlanner.show_placing_decision(b = obj.container-1, theta = obj.theta, hm_pre_pub= self.hm_pre_pub, hm_post_pub= self.hm_post_pub, score_pub= self.score_pub)
            DATA['label'].append(obj.label); DATA['container'].append(obj.container); DATA['w_dim'].append(obj.w_dim); 
            DATA['pose'].append(obj.pose); DATA['theta'].append(obj.theta); DATA['pos'].append(obj.pos);
            if self.PlacingPlanner.visionType == 'real': 
                raw_input('[ PLANNER ] Object ' + obj.label + ' placed in ' + str(obj.container) +' (Enter)')
            
            if self.visionType == 'real':    
                # SS: get time before the update
                time_before_update = math.floor(time.time())
                # SS:call passivie vision (fast mode) for state update without drop location in case it is different from suggestion        
                add_command = 'fast {} add {} '.format(obj.container,obj.label) 
                self.getPassiveVisionEstimate(add_command, obj.container)
                # SS: enable the the target bin passive vision   
                dirty_bins=['1']*4
                dirty_bins[obj.container] = '0'
                update_dirty_bins_msg = 'dirty ' + dirty_bins[0] + dirty_bins[1] + dirty_bins[2] + dirty_bins[3]
                self.getPassiveVisionEstimate(update_dirty_bins_msg, 0)
                # SS: disable all bins passive vision right after 
                time.sleep(2)
                self.getPassiveVisionEstimate('dirty 1111', 0)
                # SS: query passive vision till state has been updated
                
                passive_vision_state = self.getPassiveVisionEstimate('', obj.container)
                e.db('Vision is not yet updated, clean time according planner: ', time_before_update, ' according vision: ', passive_vision_state.heightmap_timestamp)
                while (long(passive_vision_state.heightmap_timestamp) < time_before_update):
                    e.db('Vision is not yet updated, clean time according planner: ', time_before_update, ' according vision: ', passive_vision_state.heightmap_timestamp)
                    passive_vision_state = self.getPassiveVisionEstimate('', obj.container)
                    time.sleep(0.5)
                # SS: Fetch the updated height map
                self.PlacingPlanner.update_real_height_map(obj.container-1)
                # # SS: store the placing index in container for later reference
                # self.placing_index_in_container.append(obj_in_bin_index)
        self.getPassiveVisionEstimate('dirty 0000', 0)        
        # SS: Get all object state in all storage system from passive vision 
        # SS: Loop though all container get the poses
        # print('[PLANNER] Get all object pose in all storage system from passive vision.')
        # all_object_pose_in_bin = []
        # print self.num_bins
        # for bin_id in range(self.num_bins):
        #     passive_vision_state = self.getPassiveVisionEstimate('', bin_id)
        #     object_pose_in_bin = np.asarray(passive_vision_state.state_object_pose)
        #     all_object_pose_in_bin.append(object_pose_in_bin.reshape(len(object_pose_in_bin)/7, 7))

        # # SS: Loop though all object find the poses
        # for i in range(len(self.objects)):
        #     print self.objects[i].container
        #     print self.placing_index_in_container[i]
        #     print all_object_pose_in_bin[self.objects[i].container]
        #     self.objects[i].pose = all_object_pose_in_bin[self.objects[i].container][self.placing_index_in_container[i],:]

        outfilename = os.environ['ARC_BASE']+'/input/interactive_placing.json'
        with open(outfilename, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
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
            self.clean_bin(obj.container)
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
        self.getBestGraspingPoint()
        #Execute the best primitive
        doing_suction = 'suction-tote' in self.primitives and self.suction_point is not None and self.decideDrawPrimitive()
        if 'grasp-tote' not in self.primitives or doing_suction: 
            self.best_primitive = 'suction-tote'
            e.db('-----------------------------')
            e.db('        RUNNING SUCTION      ')
            e.db('-----------------------------')
            self.run_suction()
        else:
             self.best_primitive = 'grasp-tote'
             e.db('-----------------------------')
             e.db('       RUNNING GRASPING      ')
             e.db('-----------------------------')
             self.run_grasping()

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

    def baseline_logging(self): 
        return False #This function is asked whether to stop the experiment, in baseline we don't stop
    
    def decideDrawPrimitive(self):
        #Returns True when suction is selected as the best primitive
        if self.grasp_point is None:
            return True
        if self.suction_point is None:
            return False
        suction_score = copy.deepcopy(self.suction_score)
        grasp_score = copy.deepcopy(self.grasp_score)
        if len(self.toteObj) >= 10:
            grasp_score *= 0.5
        if len(self.bad_suction_points) > 5:
            suction_score *= 0.5
        return suction_score > grasp_score

    def getBestSuctionPoint(self, container = None):
        '''Get the best suction point and correct the normal orientation'''
        self.suction_point, self.suction_score, self.suction_id = self.GetSuctionPoints(number_points=1, container=container)
        if self.suction_point is None:
            e.db('THERE ARE NO SUCTION POINTS')
            return 
        
        # Forcefully aim 2cm below suction point
        self.suction_point[2] = self.suction_point[2] - 0.02
        # TODO: now normal is considered fixed at 0,0,1
        self.suction_point[3] = 0
        self.suction_point[4] = 0
        self.suction_point[5] = 1
        self.visualize_suction_point()
        self.suction_point = list(self.suction_point)
        e.db('Best suction point:', self.suction_point, 'with score: ', self.suction_score, ' and id ', self.suction_id)

    def getBestGraspingPoint(self, container = None):
        '''Get the best grasp point considering both grasp and flush. 
            Remove those that are in collision.'''
        if container == None:
            container = self.tote_ID
        
        self.GetGraspPoints(num_points=100)
        #If no grasp or flush point available:
        self.execution_possible = False
        if self.num_pick_proposals == 0:
            self.grasp_point = None
            self.grasp_score = 0
            return
        
        #Find the best point that it is not in collision:
        num_attempts = 0
        num_it = 0
        while not self.execution_possible and num_attempts < 20 and num_it < self.num_pick_proposals: #Each time try at most 20 attempts
            self.grasp_point = copy.deepcopy(list(self.all_pick_proposals[num_it][:]))
            try:
                self.visualize_grasping_proposals(np.array([self.grasp_point]), True)
            except:
                print 'processing flush grasp not visualizing'
            num_it += 1
            #If we already know it is a bad point, we do not try it again
            if self.grasp_point in self.bad_grasping_points:
                return
            #Check if in collision
            num_attempts += 1
            if len(self.grasp_point) == self.param_flush:
                checked_output = flush_grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=False,
                                             binId=container, flag=0, withPause=False)
            else:
                checked_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=False,
                                       binId=container, flag=0, withPause=False)
            if checked_output['execution_possible']:
                self.grasp_score = copy.deepcopy(self.all_pick_scores[num_it])
                e.db('Best grasp point:', self.grasp_point, ' with score: ', self.grasp_score)
                return
            self.bad_grasping_points.append(self.grasp_point)
            self.bad_grasping_times.append(-1) 
        e.db('NONE OF THE GRASPING POINTS WORK')
        self.grasp_point = None
        self.grasp_score = 0
        return


    def call_passive_vision(self, bin_id=0):
        '''Call passive vision, log file paths, and return suction points and grasp proposals'''
        e.db('Calling passive vision system for suction and grasp proposals. The bin considered is: ', bin_id)
        self.passive_vision_state = self.getPassiveVisionEstimate('', bin_id)
        e.db('Clean time according planner: ', self.time_since_clean[bin_id], ' according vision: ', max(self.passive_vision_state.suction_timestamp, self.passive_vision_state.grasp_timestamp))
        while long(self.passive_vision_state.suction_timestamp) < self.time_since_clean[bin_id] or long(self.passive_vision_state.grasp_timestamp) < self.time_since_clean[bin_id]:
            e.db('Vision is not yet updated, clean time according planner: ', self.time_since_clean[bin_id], ' according vision: ', max(self.passive_vision_state.suction_timestamp, self.passive_vision_state.grasp_timestamp))
            time.sleep(0.5)
            self.passive_vision_state = self.getPassiveVisionEstimate('', bin_id)
        e.db('Received suction and grasp proposals.')
        #Suction
        self.all_suction_points = np.asarray(self.passive_vision_state.suction_points)
        self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/self.param_suction, self.param_suction)
        self.suction_object_list = np.asarray(self.passive_vision_state.suction_object_list)
        self.visualize_suction_points()
        #Grasp
        self.all_grasp_proposals = np.asarray(self.passive_vision_state.grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/self.param_grasping, self.param_grasping)
        self.grasp_object_list = np.asarray(self.passive_vision_state.grasp_object_list)
        self.visualize_grasping_proposals(self.all_grasp_proposals, False)
        #FlushGrasp
        self.all_flush_proposals = np.asarray(self.passive_vision_state.flush_grasp_proposals)
        self.all_flush_proposals = self.all_flush_proposals.reshape(len(self.all_flush_proposals)/self.param_flush, self.param_flush)
        self.flush_grasp_object_list = np.asarray(self.passive_vision_state.flush_grasp_object_list)
        
        #If doing picking, we need to filter them given the goal objects
        if self.task == 'picking':  
                self.pickingProposalsFilter()
        # Log saved image files and camera intrinsics files
        if self.experiment:
            self.log_file.write('passive_vision_files {}\n'.format(self.passive_vision_state.file_paths[0]))

        #Sorting all points:  #TODO: ask vision if this is still needed
        suction_permutation = self.all_suction_points[:,self.param_suction-1].argsort()[::-1]
        self.all_suction_points = self.all_suction_points[suction_permutation]
        self.suction_object_list = self.suction_object_list[suction_permutation]
        
        grasp_permutation = self.all_grasp_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]
        self.grasp_object_list = self.grasp_object_list[grasp_permutation]
        
        flush_permutation = self.all_flush_proposals[:,self.param_flush-1].argsort()[::-1]
        self.all_flush_proposals = self.all_flush_proposals[flush_permutation]
        self.flush_object_list = self.flush_object_list[flush_permutation]
        
    def CallActiveVision(self, bin_id):
        '''Call Active Vision and predict the name of the object. Add log
        @param bin_id Bin to perform vision on
        @return predicted_object_name Guesses name of object'''
        if self.task == 'stowing':
            bin_id = 1
        if self.in_simulation:
            return self.callFakeActiveVision(bin_id)
        e.db('Calling active vision system to recognize object in gripper.')
        getActiveVisionRecognition = rospy.ServiceProxy('/active_vision/recognize', active_vision.srv.recognition)
        active_vision_data = getActiveVisionRecognition(bin_id, self.weight_info[bin_id]['weights']/1000.0, self.containerObjNames(bin_id))
        highest_confidence = 0
        predicted_object_idx = 0
        e.db('object_confidence_with_weights: ', active_vision_data.object_confidence_with_weights)
        e.db('object_name: ', active_vision_data.object_name)
        for i in range(0, len(active_vision_data.object_confidence_with_weights)):
            if active_vision_data.object_confidence_with_weights[i] > highest_confidence:
                highest_confidence = active_vision_data.object_confidence_with_weights[i]
                predicted_object_idx = i
        predicted_object_name = active_vision_data.object_name[predicted_object_idx]
        if self.experiment:
            self.log_file.write('active_vision_files {}\n'.format(active_vision_data.file_paths[0]))
            self.log_file.write('active_vision_prediction {}\n'.format(predicted_object_name))
        return predicted_object_name, active_vision_data.bbox_info


    def json_output(self):
        if self.task == 'stowing':
            self.outfilename = os.environ['ARC_BASE']+'/output/stow_result.json'
        else:
            self.outfilename = os.environ['ARC_BASE']+'/output/pick_result.json'
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
        with open(self.outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
            
    def GetSuctionPoints(self, number_points, container = None):
        ''' Call vision and do some math on it (?) to return maximally
        number_points number of suction points
        @param number_points Number of suction points to return
        @return suction_points Suction points with their scores'''
        if container == None: #Stowing task
            container = self.tote_ID
        param_suction = self.param_suction
        e.db(container=container)
        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](container)            
        try_get_points = 0
        while (len(self.all_suction_points)+len(self.all_grasp_proposals)+len(self.all_flush_proposals)) == 0 and try_get_points < 4:  #Try to find suction points 4 times
            try_get_points += 1
            self.passiveVisionTypes[self.visionType](container)
        if try_get_points >= 4:
            e.db('There are no proposals at all')
        e.db('Recieved suction proposals.', theme='vision')
        
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
                if angle2gravity < 40: # If angle to gravity direction is less than 20 degrees
                    updown_suction_points_indices.append(i)
            updown_suction_points = np.zeros((len(updown_suction_points_indices), param_suction))
            updown_suction_ids = np.zeros(len(updown_suction_points_indices))
            for i in range(0, len(updown_suction_points_indices)):
                updown_suction_points[i] = self.all_suction_points[updown_suction_points_indices[i], :]
                updown_suction_ids[i] = self.suction_object_list[updown_suction_points_indices[i]]
            if len(updown_suction_points) > 0:
                self.all_suction_points = updown_suction_points
                self.suction_object_list = updown_suction_ids

            ## Remove suction points that are repeated:
            not_bad_suction_points_indices = []
            print 'bad_points: ', self.bad_suction_points
            for i in range(0, self.all_suction_points.shape[0]):
                is_bad = False
                suction_point = list(self.all_suction_points[i, 0:param_suction-1])
                for j,bad_point in enumerate(self.bad_suction_points):
                    if time.time() - self.bad_suction_times[j] > 60*1: #self.bad_suction_times[j] < time.time() + 60*3: #We only care about things that happened in the last 3 minutes
                        self.bad_suction_points.pop(j)
                        self.bad_suction_times.pop(j)
                        break
                    if np.linalg.norm(np.array(bad_point[0:2]) - np .array(suction_point[0:2]))< 0.025: #only looking position, not angle
                        is_bad = True
                        e.db('..............................................................................................')
                        e.db('Removed suction point: ', suction_point[0:2], ' because of previous bad point: ', bad_point[0:2])
                        break
                if not is_bad:
                    not_bad_suction_points_indices.append(i)
            not_bad_suction_points = np.zeros((len(not_bad_suction_points_indices), param_suction))
            not_bad_suction_ids = np.zeros(len(not_bad_suction_points_indices))
            for i in range(0, len(not_bad_suction_points_indices)):
                not_bad_suction_points[i] = self.all_suction_points[not_bad_suction_points_indices[i], :]
                not_bad_suction_ids[i] = self.suction_object_list[not_bad_suction_points_indices[i]]
            if len(not_bad_suction_points) > 0:
                self.all_suction_points = not_bad_suction_points
                self.suction_object_list = not_bad_suction_ids

        if self.visionType == 'virtual':
            self.suction_object_list = ['Null']*len(self.all_suction_points)
        if len(self.all_suction_points) == 0:
            return None, None  #Return nothing
        if number_points == 1:
            #print self.all_suction_points
            return self.all_suction_points[0, 0:param_suction-1], self.all_suction_points[0, param_suction-1], self.suction_object_list[0]
        return self.all_suction_points[:, 0:param_suction-1], self.all_suction_points[:, param_suction-1], self.suction_object_list

    def GetGraspPoints(self, num_points, container = None):
        '''Call passive vision to get grasp points.'''
        if container == None:
            container = self.tote_ID

        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](container)
        e.db('[Vision] Received grasp proposals.')
        if self.visionType == 'virtual':
            self.grasp_object_list = ['Null']*len(self.all_grasp_proposals)
            self.flush_object_list = ['Null']*len(self.all_flush_proposals)
        #Add grasp proposals if possible
        num_points_grasp = min(num_points, len(self.all_grasp_proposals))        
        if len(self.all_grasp_proposals) > 0 and not self.only_flush:
            self.all_pick_proposals = list(self.all_grasp_proposals[0:num_points_grasp, 0:self.param_grasping])
            self.all_pick_scores = list(self.all_grasp_proposals[0:num_points_grasp, self.param_grasping-1])
            self.all_pick_ids = list(self.grasp_object_list[0:num_points_grasp])
        else: 
            self.all_pick_proposals = []
            self.all_pick_scores = []
            self.all_pick_ids = []
        #Add flush proposals if possible
        num_points_flush = min(num_points, len(self.all_flush_proposals)) 
        if len(self.all_flush_proposals) > 0 and not self.only_grasp and container < 2:
            self.all_pick_proposals += list(self.all_flush_proposals[0:num_points_flush, 0:self.param_flush])
            self.all_pick_scores += list(self.all_flush_proposals[0:num_points_flush, self.param_flush-1])
            self.all_pick_ids += list(self.flush_object_list[0:num_points_flush])
            
            
        '''
        pick_permutation = self.all_pick_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]
        self.grasp_object_list = self.grasp_object_list[grasp_permutation]
        '''
        self.all_pick_scores.sort(reverse=True)
        self.all_pick_proposals.sort(key=lambda x: x[-1], reverse=True)
        self.num_pick_proposals = len(self.all_pick_scores)

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

    def runActiveID(self, container = None):
        '''Run either real or fake vision to get the ID from active vision'''
        if container == None:
            container = self.tote_ID
        if self.visionType == 'virtual':
            self.active_vision_ID, self.bbox_info = self.callFakeActiveVision(container)
        else:
            self.active_vision_ID, self.bbox_info = self.CallActiveVision(container) 

    def run_suction(self, container = None):
        '''Activate sution and log whether successful or not.'''
        if self.suction_point is None:
            e.db('It was suppose to do grasping, but there is no grasp proposal')
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
        if self.in_simulation:
            self.callFakeSuction(prob=0.8, container = container)
            return
        e.db(SuctionPositionTarget=self.suction_point[0:3])
        e.db(SuctionNormal=self.suction_point[3:6])
        #Prepare for suction
        goToHome.prepGripperSuction()
        self.suction_output = suction_down_simple(listener = self.listener, br=self.br, withPause=self.withPause,
                        suction_position_target_list=[self.suction_point[0:3]],
                        surface_normal_list=[self.suction_point[3:6]], flag=0,
                        bin_id=container,viz_pub=self.viz_array_pub)

        self.primitive_output = self.suction_output
        self.plan_possible = self.suction_output['success_flag'] #suction_output['plan_possible']
        self.suction_check = self.suction_output['suction_check'] #self.suction_check = raw_input('[HELP] suck or not')=='true'
        #Go home
        #suction_down_simple(listener = self.listener, br=self.br, withPause=self.withPause, flag=1, bin_id=container,viz_pub=self.viz_array_pub)
        self.execution_possible = (self.plan_possible and self.suction_check)
        e.db(plan_possible=self.plan_possible)
        e.db(suction_check=self.suction_check)

    def run_grasping(self, container = None):
        '''Run either flush or normal grasping at grasp_point and log whether
        execution is successful or not
        Assumes self.grasp_point, self.grasp_score have been computed'''
        if self.grasp_point is None:
            e.db('It was suppose to do grasping, but there is no grasp proposal')
            self.execution_possible = False
            self.primitive_output = None
            self.primitive_point = None
            self.primitive_score = 0
            self.suction_check = False
            return
        if container == None:
            container = self.tote_ID
        self.dirty_bin(container)
        self.primitive_score = self.grasp_score
        self.primitive_point = self.grasp_point
        if self.in_simulation:
            e.db('CALLING VIRTUAL GRASPING')
            self.callFakeGrasping(prob=0.8) #TODO
            return
        e.db(grasp_point=self.grasp_point)
        #e.db(LENGTH=len(self.grasp_point))
        #e.db(PARAMFLUSH=self.param_flush)
        #Prepare for grasping
        
        goToHome.prepGripperPicking()
        if len(self.grasp_point) == self.param_flush:
            self.grasping_output = flush_grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                               isExecute=self.isExecute, binId=container, flag=0,
                                               withPause=self.withPause)
        else:
            self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                         isExecute=self.isExecute, binId=container, flag=0,
                                         withPause=self.withPause)
        #Go home
        grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
              binId=container, flag=1, withPause=self.withPause)
        #Read the primitive output
        self.pick_possible = self.grasping_output['grasp_possible']
        self.plan_possible = self.grasping_output['plan_possible']
        self.execution_possible = self.grasping_output['execution_possible']
        self.collision_detected = self.grasping_output['collision']
        self.grasping_output['graspPose'] = list(self.grasping_output['graspPose'])
        self.primitive_output = self.grasping_output
        e.db(execution_possible=self.execution_possible)

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

    def planned_place(self):
        '''Call placing_planner17 based on height. Compute height map and
        either place via suction or drop in a specific location (from planner)'''
        placedObj = self.objects[self.placedObj]
        container_before_placing = copy.deepcopy(placedObj.container)
        self.pickedObjects.append(placedObj.label)
        e.db('Object that have been attempted to be placed: ', self.pickedObjects)
        self.PlacingPlanner.box_placing = False
        if self.task == 'picking':
            #Choose box for it
            self.choose_box_for_object(placedObj)
            #If the object is a goal:
            if placedObj.container >= self.num_containers: #That means the object is a goal and goes into a box
                self.PlacingPlanner.box_placing = True
                limit_x = 0.5*(rospy.get_param("/tote/length")-rospy.get_param('/bin%d/length'%placedObj.container))
                limit_x = max(0, int(math.floor(limit_x/self.PlacingPlanner.disc)))
                limit_y = 0.5*(rospy.get_param("/tote/width")-rospy.get_param('/bin%d/width'%placedObj.container))
                limit_y = max(0, int(math.floor(limit_y/self.PlacingPlanner.disc)))
                self.PlacingPlanner.limit_x = limit_x #10
                self.PlacingPlanner.limit_y = limit_y #5 #Box dependent
        #Update used dimensions of the object to match with bounding box from vision
        bbox_size = self.bbox_info[7:10]
        placedObj.update_w_dim_with_bbox(bbox_size)
        #Update HM
        if self.PlacingPlanner.visionType == 'real' and not self.PlacingPlanner.box_placing: #TODO: is this the best place? 
            self.PlacingPlanner.update_real_height_map()
        #Find best place object
        if not self.PlacingPlanner.box_placing:
            placing_score = self.PlacingPlanner.place_object_local_best(self.placedObj) #Change container and pos placedObj,  tries 3 bins
        else: 
            placing_score = self.PlacingPlanner.place_object_local_best(self.placedObj, containers = [placedObj.container-1]) #Change container and pos placedObj
        
        self.PlacingPlanner.update_pose_placed_obj(self.placedObj) #World pose
        self.PlacingPlanner.show_placing_decision(b = placedObj.container-1, theta = placedObj.theta , hm_pre_pub= self.hm_pre_pub, hm_post_pub= self.hm_post_pub, score_pub= self.score_pub)
        ####  TODO: Recall that objects in planner might not be de same as in pacing
        e.db('Placing ', placedObj.label ,'on bin ', placedObj.container, ' position ', placedObj.pos, 'pose' , placedObj.pose, 'given the score: ', placing_score)
        #Get bounding box
        drop_pose = placedObj.pose
        #Run primitive
        self.dirty_bin(placedObj.container)
        self.clean_bin(container_before_placing) #It has to be the old container!
        
            
        if self.best_primitive == 'suction-tote':
            suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=2,
                                bin_id=placedObj.container, suction_position_target_list = [drop_pose],
                                rel_pose=self.rel_pose,BoxBody=self.BoxBody,place_pose=drop_pose,viz_pub=self.viz_array_pub)
            if self.PlacingPlanner.box_placing: 
                collision_free_placing(binId=self.tote_ID, listener=self.listener, isSuction=True)
        else:
            grasping_output_drop=grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                       isExecute=self.isExecute, objId=self.pickedLabel,
                                       binId=placedObj.container, flag=2, withPause=self.withPause,
                                       rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose)
            if self.PlacingPlanner.box_placing: 
                collision_free_placing(binId=self.tote_ID, listener=self.listener, isSuction=False)
        if self.PlacingPlanner.box_placing:
            self.PlacingPlanner.limit_x = 6
            self.PlacingPlanner.limit_y = 2 #Manually adjusted

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
    ### VISUALIZATIONS ###
    ######################
    def visualize_suction_points(self):
        # self.all_suction_points  n*7 np.array
        n = self.all_suction_points.shape[0]
        color = (0, 1, 0, 1)  # rgba
        scale = 0.03
        markers_msg = MarkerArray()
        markers_msg.markers.append(createDeleteAllMarker('suction_points'))
        subrate = 1
        if n > 100:
            subrate = n/100
        for i in xrange(0,n,subrate):
            suction_point = self.all_suction_points[i,:]
            start = suction_point[0:3]
            direc = suction_point[3:6]
            end = start + direc/np.linalg.norm(direc) * scale
            m = createArrowMarker(start, end, color, marker_id = i, ns = 'suction_points')
            markers_msg.markers.append(m)
        self.viz_array_pub.publish(markers_msg)
        #pauseFunc(True)

    def visualize_suction_point(self):
        # self.suction_point  len 7 
        color = (0, 0, 1, 1)  # rgba
        scale = 0.1
        markers_msg = MarkerArray()
        suction_point = self.suction_point
        start = suction_point[0:3]
        direc = suction_point[3:6]
        end = start + direc/np.linalg.norm(direc) * scale
        m = createArrowMarker(start, end, color, marker_id = 10000, ns = 'suction_points')
        markers_msg.markers.append(m)
        self.viz_array_pub.publish(markers_msg)
        #pauseFunc(True)

    def visualize_grasping_proposals(self, proposals, is_selected = False):
        
        # visualize all_grasp_proposals 2d list (n * 12)
        def get_picking_params_from_12(objInput):  # copied from picking17
            #~define variables
            grasp_begin_pt=np.array(objInput[0:3])
            hand_Z=np.array(objInput[3:6])
            
            #~define grasp pos
            grasp_depth=np.array(objInput[6])
            graspPos=grasp_begin_pt + hand_Z*grasp_depth
            grasp_width=np.array(objInput[7])
            
            #~define hand frame
            hand_X=np.array(objInput[8:11])
            hand_Y=np.cross(hand_Z, hand_X)

            return graspPos, hand_X, hand_Y, hand_Z, grasp_width
        
        n = proposals.shape[0]
        
        color = (0, 1, 1, 1)  # rgba
        if is_selected:
            color = (1, 1, 0, 1)
        color_bar = (0.5, 0.5, 0.5, 0.5)  # rgba
        scale = (0.001,0.02,0.1)
        markers_msg = MarkerArray()
        #if not is_selected:  # don't delete other candidate
        #    markers_msg.markers.append(createDeleteAllMarker('pick_proposals'))
        subrate = 1
        if n > 5:
            subrate = n/5
        for i in xrange(0,n,subrate):
            pick_proposal = proposals[i,:]
            graspPos, hand_X, hand_Y, hand_Z, grasp_width = get_picking_params_from_12(pick_proposal)
            scale_bar = (grasp_width,0.003,0.1)
            #import ipdb; ipdb.set_trace()
            rotmat = np.vstack((hand_X, hand_Y, hand_Z, np.zeros((1,3)))).T
            rotmat = np.vstack((rotmat, np.array([[0,0,0,1]])))
            quat = quat_from_matrix(rotmat)
            m1 = createCubeMarker2(rgba = color, scale = scale, offset = tuple(graspPos+hand_X*grasp_width/2), orientation= tuple(quat), marker_id = i*3, ns = 'pick_proposals')
            m2 = createCubeMarker2(rgba = color, scale = scale, offset = tuple(graspPos-hand_X*grasp_width/2), orientation= tuple(quat), marker_id = i*3+1, ns = 'pick_proposals')
            m3 = createCubeMarker2(rgba = color_bar, scale = scale_bar, offset = tuple(graspPos), orientation= tuple(quat), marker_id = i*3+2, ns = 'pick_proposals')
            
            markers_msg.markers.append(m1)
            markers_msg.markers.append(m2)
            markers_msg.markers.append(m3)
        self.viz_array_pub.publish(markers_msg)

    ######################
    ### MAIN FUNCTIONS ###
    ######################
    def run_picking(self):
        '''Main picking function loop.'''
        #Initialize robot state
        goToHome.goToARC(slowDown=self.goHomeSlow)
        
        self.picking_preparation[self.decider]()
        
        self.time_begin = time.time()
        self.time_end = self.time_begin + self.duration_of_contest
        
        self.all_suction_points = None
        self.num_attempts_iterations = 0 
        while self.goals_left > 0 and time.time() < self.time_end:
            e.db(GOALS_LEFT=self.goals_left)
            raw_input('Ready for picking a new object?')
            self.printObjects()
            e.db('The time left is: ', self.time_end - time.time())
            if self.num_attempts_iterations >= 1: 
                self.all_suction_points = None
                self.num_attempts_iterations = 0

            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)

            #Run the pick baseline
            self.pick[self.decider]()
            
            #Action executed in self.best_container. Check weight:
            self.weight_info[self.best_container] = self.weightSensor.readWeightSensor(item_list=self.toteObjNames(), withSensor=self.withSensorWeight, binNum=self.best_container, givenWeights= -11*int(self.execution_possible))

            if self.weight_info[self.best_container]['weights'] > 10: 
                self.execution_possible = True
            e.db('Detected weight:',  self.weight_info[self.best_container]['weights'], ' in container: ', self.best_container, ' the pick is good?(True = Yes)', self.execution_possible)
            
            #Run active vision
            if self.execution_possible or self.forceSucc:
                e.db('Running active ID')
                self.runActiveID(self.best_container)
                self.pickedLabel = self.active_vision_ID
                webpages.webDisplay(self.pickedLabel)
                if self.pickedLabel == 'empty':
                    self.execution_possible = False
                e.db('----------------------------')
                e.db('Object picked according active vision: ', self.pickedLabel)
                e.db('----------------------------')
                self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info,viz_pub=self.viz_array_pub,listener=self.listener)

            e.db('----------------------------')
            e.db(Execution_possible = self.execution_possible)
            e.db('----------------------------')

            self.json_output()
            if (self.execution_possible or self.forceSucc): #success
                self.num_attempts_iterations = 1 
                
                ## Check object in ShelfObj, remove it and update json
                for i, x in enumerate(self.shelfObj):
                    if self.objects[x].label == self.pickedLabel and self.objects[x].container == self.best_container:
                        self.placedObj = x
                        container_before_placing = copy.deepcopy(self.objects[x].container)
                        #Placing the object
                        self.placer[self.decider]()
                        goToHome.goToARC(slowDown = self.goHomeSlow)
                        obj = self.objects[x]
                        #Check placing succeded using weight changes
                        if obj.container != container_before_placing:
                            for j in range(1,9): #TODO: avoid weight failure put 8 instead 9
                                if j == 5 or j == 7:
                                    self.weight_info_after_place[j] = self.weight_info_after_place[j-1]
                                    continue
                                self.weight_info_after_place[j] = self.weightSensor.readWeightSensor(item_list=[], withSensor=self.withSensorWeight, binNum=j, givenWeights=obj.weight*1000)

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
                                    
                        if obj.container > 3: #Not in bins, but boxes
                            self.shelfObj.pop(i)
                            if self.visionType == 'real':
                                rm_command = '{} rm {}'.format(container_before_placing, obj.label)
                                self.getPassiveVisionEstimate(rm_command, container_before_placing)               
                        else: 
                            if self.visionType == 'real':               
                                if obj.container == container_before_placing:
                                    mv_command = '{} mv {}'.format(obj.container, obj.label)
                                    self.getPassiveVisionEstimate(mv_command, obj.container)
                                else:
                                    rm_command = '{} rm {}'.format(container_before_placing, obj.label)
                                    self.getPassiveVisionEstimate(rm_command, container_before_placing)               
                                    add_command = '{} add {}'.format(obj.container, obj.label)
                                    self.getPassiveVisionEstimate(add_command, obj.container)
                        self.update_goals_after_pick(self.objects[x])
                        break #We do not want to look at repeated objects
            else: #Fail
                if self.best_primitive == 'suction-tote':
                    suction.stop()
                    #suction.pump_stop()
                    self.bad_suction_points.append(self.primitive_point)
                    self.bad_suction_times.append(time.time())
                else:
                    goToHome.prepGripperPicking()
                    self.bad_grasping_points.append(self.primitive_point)
                    self.bad_grasping_times.append(time.time())
                self.num_attempts_iterations += 1  #If lower than 1 we should try to pick without doing vision
                #Prevent problems with active vision:
                goToHome.goToARC(slowDown = self.goHomeSlow)
                self.clean_bin(self.best_container)
            self.json_output()
        
        #Finishing picking
        goToHome.goToARC(slowDown = self.goHomeSlow)
        e.db("Planner is done")
        
    def run_stowing(self):
        '''Main stowing function loop. This will loop until all objects are cleared
        or the time limit is hit. Generically calls each action, with the mode
        being indicated by the self.decider. Stows objects and logs all data'''
        self.time_begin = time.time()
        self.time_end = self.time_begin + self.duration_of_contest
        #Initialize robot state
        goToHome.goToARC(slowDown=self.goHomeSlow)
                
        self.preparation[self.decider]()
        self.json_output()
        self.all_suction_points = None
        self.num_attempts_iterations = 0
        while len(self.toteObj) > 0 and time.time() < self.time_end:
            #raw_input('Next try')
            self.initial_seq_time = time.time()
            e.db('The time left is: ', self.time_end - self.initial_seq_time)
            e.db(num_attempts_iterations = self.num_attempts_iterations)
            if self.num_attempts_iterations >= 1: #By now we always want to call vision
                self.all_suction_points = None
                self.num_attempts_iterations = 0
       
            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)
   
            #Execute best primitive
            self.draw[self.decider]()
        
            #Use weight to check
            self.weight_info[self.tote_ID] = self.weightSensor.readWeightSensor(item_list=self.toteObjNames(), withSensor=self.withSensorWeight, binNum=self.tote_ID, givenWeights=-11*int(self.execution_possible))

            if self.weight_info[self.tote_ID]['weights'] > 10: 
                self.execution_possible = True 
            e.db('Detected weight:',  self.weight_info[self.tote_ID]['weights'], ' execution seems possible?(True = Yes)', self.execution_possible)

            # Query the user to determine if the experiment succeeded
            if self.human_supervision:
                self.execution_possible = self.GetHumanInputOnExperimentStatus()

            e.db('----------------------------')
            e.db(execution_possible = self.execution_possible)
            e.db('----------------------------')


            if self.execution_possible or self.forceSucc:
                e.db('Running active ID')
                ## Moure a bin 1
                if self.best_primitive == 'grasp-tote':
                    grasp(objInput=[], listener=self.listener, br=self.br, isExecute=self.isExecute,
                          binId=1, flag=1, withPause=self.withPause)
                else:
                    suction_down_simple(listener = self.listener, br=self.br, withPause=self.withPause, flag=1, bin_id=1,viz_pub=self.viz_array_pub)
                
                #try:
                self.runActiveID()
                self.pickedLabel = self.active_vision_ID
                webpages.webDisplay(self.pickedLabel)
                if self.pickedLabel == 'empty':
                    self.execution_possible = False
                e.db('Object picked according active vision: ', self.pickedLabel, ' and the bbox_info is: ',self.bbox_info)
                bbox_pose_ros = list(self.bbox_info[4:7] + self.bbox_info[0:4])  # change matlab quat to our convention [x,y,z, qx,qy,qz,qw]
                pubFrame(self.br, pose=bbox_pose_ros)
                self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info,viz_pub=self.viz_array_pub,listener=self.listener)
                e.db('self.rel_pose',self.rel_pose)
                e.db('self.BoxBody',self.BoxBody)
                #self.rel_pose, self.BoxBody=vision_transform_precise_placing(self.bbox_info, listener= self.listener)

            if self.human_supervision:
                e.db('ID given by active vision: ', self.active_vision_ID)
                self.pickedLabel = self.GetHumanToLabelObject(self.execution_possible)

            if self.experiment:
                self.log_file.write('pick_successful: {}\n\n\n'.format(self.execution_possible))

            self.json_output()
            e.db('----------------------------------------------')
            e.db(Execution_possible = self.execution_possible)
            e.db('----------------------------------------------')
            if (self.execution_possible or self.forceSucc): #success
                self.fails_in_row = 0
                self.num_attempts_iterations = 1 
                ##Place object and check final position
                for i, x in enumerate(self.toteObj):
                    if self.objects[x].label == self.pickedLabel:
                        self.placedObj = x
                        container_before_placing = copy.deepcopy(self.objects[x].container)
                        #Running the place
                        self.placer[self.decider]()
                        obj = self.objects[x]
                        #Update weight sensors outputs
                        for j in range(4):
                            self.weight_info_after_place[j] = self.weightSensor.readWeightSensor(item_list=[], withSensor=self.withSensorWeight, binNum=j, givenWeights=obj.weight*1000) 
                        e.db('detected weight',  self.weight_info_after_place[obj.container]['weights'], 'while the weight of the placed object is: ', obj.weight*1000)
                        
                        if self.weight_info_after_place[obj.container]['weights'] > -5:  #Virtual mode never enters here...
                            e.db('It feels like the object was not placed where it was supposed to be!')
                            #The object was not placed as expected
                            max_weight_change = -10  #Because the object has been placed there, the weight should be negative 
                            for j in range(4): #Look into the bins and boxes
                                if max_weight_change > self.weight_info_after_place[j]['weights']:
                                    max_weight_change = self.weight_info_after_place[j]['weights']
                                    obj.container = j
                            if max_weight_change == -10:  #In that case, the object remained where it was before
                                obj.container = container_before_placing
                                
                        if obj.container != self.tote_ID:
                            self.shelfObj.append(x)
                            self.toteObj.pop(i)                                    
                            if self.visionType == 'real':               
                                add_command = '{} add {} {} {} {}'.format(obj.container,
                                    obj.label, obj.pose[0], obj.pose[1], obj.pose[2])
                                self.getPassiveVisionEstimate(add_command, obj.container)
                        break #We do not want to look at repeated objects
            else: #Fail
                if self.best_primitive == 'suction-tote':
                    suction.stop() 
                    #suction.pump_stop()
                    self.bad_suction_points.append(self.primitive_point)
                    self.bad_suction_times.append(time.time())
                else:
                    goToHome.prepGripperPicking()
                    self.bad_grasping_points.append(self.primitive_point)
                    self.bad_grasping_times.append(time.time())
                self.fails_in_row += 1
                self.num_attempts_iterations += 1 
                goToHome.goToARC(slowDown = self.goHomeSlow)
                self.clean_bin(self.tote_ID)
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

    def run(self):
        #Extend the shelf
        #shelf_helper.open_shelf(withPause=self.withPause)
        #Run the right task
        '''Exeecutes only the stowing task'''
        if self.task == 'stowing':
            self.run_stowing()
        elif self.task == 'picking':
            self.run_picking()
        elif self.task == 'combined':
            self.combined_task = True
            #Start with stowing
            self.task == 'stowing'
            self.run_stowing()
            #Now do picking
            self.task == 'picking'
            self.jsonfilename = self.outfilename
            self.enter_jsonfile()
            self.run_picking()
        else:
            assert False, 'I can only do stowing or picking'

if __name__ == '__main__':
    ## 
    rospy.init_node('Planner')
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

    parser.add_option('--task', action='store', default='stowing',
        help='Task to run [stowing[def]/picking/combined]')

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
