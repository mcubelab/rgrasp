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
from placing_planner17 import PlacingPlanner
import random, subprocess, time, datetime, json, math, optparse, rospy
import scipy.stats
import random as rand
import gripper
import tf
#from ik.ik import *
from ik.helper import get_params_yaml
import numpy as np
import tf.transformations as tfm
from manual_fit.srv import *
import os
from collision_detection.collisionHelper import collisionCheck
import suction 
try:
    import passive_vision.srv
    import active_vision.srv
    import realsense_camera.srv
except:
    print 'FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL'
try:
    import sys
    sys.path.append('/home/mcube/arc/catkin_ws/src/weight_sensor/src')
    import ws_prob
except:     
    print 'FAILED TO IMPORT WEIGHT, WILL ONLY RUN IN VIRTUAL'

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
from ik.helper import get_obj_vol, rotmatZ, getBinMouth, get_obj_dim, matrix_from_xyzquat, pauseFunc, find_object_pose_type_2016, find_object_pose_type_2016_short, getObjCOM, quat_from_matrix, get_bin_cnstr, in_or_out, moveGripper, getBinMouthAndFloor
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
                                   'virtual' : self.callFakePassiveVision}
                                    #TODO: 'file': percept.perceptAPC2016
        self.deciderTypes = {'experiment' : None,
                             'baseline' : None,
                             'random' : None}
        self.all_primitives = ['suction-tote', 'grasp-tote']
        self.preparation = {'experiment' : self.experiment_preparation,
                            'baseline' : self.baselinePreparation,
                            'random': self.baselinePreparation}
        self.pick = {'experiment' : self.experiment_pick,
                     'baseline' : self.baseline_pick,
                     'random': self.random_pick}
        self.placer = {'experiment' : self.planned_place,
                       'baseline' : self.planned_place,
                       'random': self.planned_place}
        #self.placer = {'experiment' : self.default_place,
        #               'baseline' : self.default_place,
        #               'random': self.default_place}
        self.store_info = {'experiment' : self.experiment_logging,
                           'baseline' : self.baseline_logging,
                           'random': self.baseline_logging}
        self.FAKE_VISION_DIR = os.environ['ARC_BASE'] + '/input/fake_dirs/fake_vision/'
        self.FAKE_GRASPING_DIR = os.environ['ARC_BASE'] + '/input/fake_dirs/fake_grasping/'
        self.FAKE_SUCTION_DIR = os.environ['ARC_BASE'] + '/input/fake_dirs/fake_suction/'
        self.param_suction = 7
        self.param_grasping = 12
        self.param_flush = 6
        self.penalty_bad_suction = 1.
        self.penalty_bad_grasping = 1.
        ##################################
        ### Assertions before starting ###
        ##################################
        assert ((not opt.forceSucc) or opt.in_simulation), 'Can only force success in simulation'
        assert opt.visionType in self.passiveVisionTypes, 'visionType should be in'+str([_ for _ in self.passiveVisionTypes])+'but its '+opt.visionType
        assert (opt.visionType == 'real' or (not opt.experiment)), 'visionType should be real during experiments'
        assert (opt.in_simulation or opt.visionType != 'virtual'), 'we should be in simulation to use virtual vision'
        assert (opt.visionType == 'virtual' or (not opt.in_simulation)), 'visionType should be virtual in simulation'
        assert (not opt.experiment or not opt.in_simulation), 'Cannot do experiments in simulation'
        assert (opt.decider in self.deciderTypes), 'Decider should be in '+str([_ for _ in self.deciderTypes])
        #Check path for jsonfile
        self.jsonfilename = opt.jsonfilename
        if not os.path.isfile(self.jsonfilename):
            self.jsonfilename = os.environ['ARC_BASE'] + '/input/' + self.jsonfilename
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
        #######################
        ### Class variables ###
        #######################
        self.tote_ID = 0
        self.num_bins = 3
        self.objects = [] #List of objects in this run [in all places]
        self.toteObj = [] #List of indices to the object list of objects in tote
        self.shelfObj = [] #List of indices to the object list of objects in shelf
        if self.experiment:
            self.init_experiment(opt) #initialize things for experiments
        else:
            self.duration_of_contest = 15*60 #15 min
            self.decider = opt.decider
        #Vision type
        self.visionType = opt.visionType
        #JSON
        self.use_last_json = False
        self.enter_jsonfile()
        #Set primitives
        self.primitives = []
        for primitive in self.all_primitives:
            if (primitive in opt.primitives) or (opt.primitives == 'all'):
                self.primitives.append(primitive)
        self.fails_in_row = 0
        ## Manipulation
        self.goHomeSlow = False
        self.primitive_version = 1
        self.dirty_bins = "0"*4 #All bins start clean

        ## ROS setup
        rospy.init_node('planner17', anonymous=True)
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.publisher = rospy.Publisher('dirty_bins', std_msgs.msg.String, queue_size=10)
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        rospy.sleep(0.5)

        #Bad points
        self.bad_suction_points = [] #list of failed attempts
        self.bad_grasping_points = []
        self.bad_suction_times = [] #list of times when we failed
        self.bad_grasping_times = []

        #Placing
        self.PlacingPlanner = PlacingPlanner()
        self.PlacingPlanner.put_objects_from_planner(self.objects)
        
        #WeightF
        self.withSensorWeight = True
        self.weightSensor = ws_prob.WeightSensor()

        ########################################
        ### Potentially not useful variables ###
        ########################################
        self.weight_info = None
        self.vision_info = None
        self.active_vision_ID = None
        e.db(decider=self.decider)
        e.db(experiment=self.experiment)


    ###############################
    ### GLOBAL HELPER FUNCTIONS ###
    ###############################
    def clean_bin(self, bin_num = -1):
        #bin_num = -1 updates all of them
        for i in range(4):
            if bin_num == -1 or bin_num == i:
                self.dirty_bins = self.dirty_bins[:i]+'0'+self.dirty_bins[i+1:]
        self.publisher.publish(std_msgs.msg.String(self.dirty_bins))

    def dirty_bin(self, bin_num = -1):
        #bin_num = -1 updates all of them
        for i in range(4):
            if bin_num == -1 or bin_num == i:
                self.dirty_bins = self.dirty_bins[:i]+'1'+self.dirty_bins[i+1:]
        self.publisher.publish(std_msgs.msg.String(self.dirty_bins))

    def enter_jsonfile(self):
        '''Extracts the info from the input jsonfile'''
        if self.experiment and (not self.use_last_json):
            self.talkWithHumanToStartExperiment()
        else:
            with open(self.jsonfilename) as data_file:
                DATA = json.load(data_file)
            ##Introduce objects in tote
            for obj in DATA['tote']['contents']:
                self.objects.append(Item(container=0))
                self.objects[-1].update_with_label(obj)
            self.toteObj = range(len(self.objects))
            ##Introduce objects in tote
            #TODO: adapt to multiple bins!
            for x in range(self.num_bins):
                for obj in DATA['bins'][x]['contents']: #Now assume single shelf
                    self.objects.append(Item(container=x+1)) #TODO: assuming one bin
                    self.objects[-1].update_with_label(obj)
            self.shelfObj = range(len(DATA['tote']['contents']), len(self.objects))
            if self.use_last_json:
                e.db(toteObjNames=self.toteObjNames)
                finished_placing = 'n'
                while finished_placing != 'y':
                    finished_placing = raw_input('Have you finished placing the objects?[y/n]')
        if not self.use_last_json:
            DATA = {}
            DATA['tote'] = {}
            DATA['tote']['contents'] = self.toteObjNames()
            DATA['bins'] = []
            for x in range(self.num_bins):
                DATA['bins'].append({'bin_id': chr(ord('A')+x), 'contents': self.shelfObjNamesGivenBin(x+1) })
            with open(os.path.join(os.environ['ARC_BASE'], 'input/last_input.json'), 'w') as outfile:
                json.dump(DATA, outfile, sort_keys=True, indent=4,
                          ensure_ascii=False)

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

    def adjustPredictionsWithFailedAttempts(self):
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

    def callFakeActiveVision(self, tote_ID):
        '''For fake active vision, pick a random label from the tote
        @param tote_ID (unused?)
        @return self.pickedLabel label of picked object'''
        self.pickedLabel = self.toteObjNames()[np.random.randint(len(self.toteObj))]
        return self.pickedLabel, None

    def callFakePassiveVision(self, tote_ID):
        '''For Fake Passive vision, read the past results of vision from a
        data file and pull out the suction and grasp proposals.
        @param tote_ID unused because vision isnt checking a tote '''
        f = self.FAKE_VISION_DIR + random.choice(os.listdir(self.FAKE_VISION_DIR))
        e.db(f=f)
        with open(os.path.join(self.FAKE_VISION_DIR, f), 'r') as infile:
            DATA = json.load(infile)

        self.all_suction_points = DATA['suction_points']
        self.all_suction_points = np.array(self.all_suction_points)
        self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/7, 7)
        self.all_suction_points = self.all_suction_points[self.all_suction_points[:, 7-1].argsort()[::-1]]

        self.all_grasp_proposals = DATA['grasp_proposals']
        self.all_grasp_proposals = np.array(self.all_grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/12, 12)

        # Check if flush proposals are in data. If so use them, otherwise populate as empy
        if 'flush_proposals' in DATA:
            self.all_flush_proposals = DATA['flush_proposals']
            self.all_flush_proposals = np.array(self.all_flush_proposals)
            self.all_flush_proposals = self.all_flush_proposals.reshape(len(self.all_flush_proposals)/self.param_flush, self.param_flush)
        else:
            self.all_flush_proposals = np.array([])
        self.adjustPredictionsWithFailedAttempts()

    def callFakeGrasping(self, prob=-1.):
        '''Call a fake grasping with some probability of success?
        @param prob Probability of grasping succeeding'''
        self.dirty_bin(self.tote_ID)
        _ = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                  binId=self.tote_ID, flag=0, withPause=False)
        f = random.choice(os.listdir(self.FAKE_GRASPING_DIR))
        with open(os.path.join(self.FAKE_GRASPING_DIR, f), 'r') as infile:
            self.grasping_output = json.load(infile)
        self.grasping_output = self.grasping_output['primitive_output']
        if self.forceSucc:
            self.grasping_output['pick_possible'] = True
        elif prob >= 0.: #Change success
            self.grasping_output['pick_possible'] = (np.random.rand() <= prob)
        self.grasping_output['execution_possible'] = self.grasping_output['pick_possible']
        self.execution_possible = self.grasping_output['execution_possible']
        self.pickedLabel = 'unknown'
        self.clean_bin(self.tote_ID)

    def callFakeSuction(self, prob=-1.):
        '''Call a fake suction with some probability of success?
        @param prob Probability of suction succeeding'''
        self.dirty_bin(self.tote_ID)
        f = random.choice(os.listdir(self.FAKE_SUCTION_DIR))
        with open(os.path.join(self.FAKE_SUCTION_DIR, f), 'r') as infile:
            self.suction_output = json.load(infile)
        self.suction_output = self.suction_output['primitive_output']
        if self.forceSucc:
            self.suction_output['success_flag'] = True
        elif prob >= 0.: #Change success
            self.suction_output['success_flag'] = (np.random.rand() <= prob)
        self.suction_output['execution_possible'] = self.suction_output['success_flag']
        self.execution_possible = self.suction_output['execution_possible']
        self.pickedLabel = 'unknown'
        self.clean_bin(self.tote_ID)
    ################
    ### BASELINE ###
    ################
    def baselinePreparation(self):
        pass

    def baseline_pick(self):
        '''Baseline picking operation. Compute the best suction and
        grasping score, then run whichever is best'''
        e.db('Objects in the tote: ', self.toteObjNames())
        #Get Best Suction Point
        self.getBestSuctionPoint()
        self.getBestGraspingPoint()
        if True: # TODO self.suction_score > self.grasp_score:
            self.best_primitive = 'suction-tote'
            #assert self.suction_point is not None
            e.db('RUNNING SUCTION')
            self.run_suction()
        else:
            self.best_primitive = 'grasp-tote'
            #assert self.grasp_point is not None
            e.db('RUNNING GRASPING')
            self.run_grasping()

    def random_pick(self):
        '''Random picking operation. Compute best suction and grasping scores
        Randomly run either operation, given that there is a suction or grasp
        point (respectively)'''
        e.db('Objects in the tote: ', self.toteObjNames())
        #Get Best Suction Point
        self.getBestSuctionPoint()
        self.getBestGraspingPoint()
        if (self.suction_score is not None and np.random.rand() > 0.5) or (self.grasp_point is None):
            self.best_primitive = 'suction-tote'
            assert self.suction_point is not None
            e.db('RUNNING SUCTION')
            self.run_suction()
        else:
            self.best_primitive = 'grasp-tote'
            assert self.grasp_point is not None
            e.db('RUNNING GRASPING')
            self.run_grasping()

    def baseline_logging(self):
        pass
    ###################
    ### EXPERIMENTS ###
    ###################

    def getBestSuctionPoint(self):
        '''Get the suction points by calling upon vision. Then adjust
        the orientation of each suction (with manual fixes). '''
        self.suction_point, self.suction_score = self.GetSuctionPoints(number_points=1)
        if self.suction_point is None:
            self.suction_point, self.suction_score = self.GetSuctionPoints(number_points=1)
        if self.suction_point is None:
            e.db('Vision can not find suction points')
            assert(False)
        # Forcefully aim 2.5cm below suction point
        self.suction_point[2] = self.suction_point[2] - 0.025
        # TODO: now normal is considered fixed at 0,0,1!!!
        self.suction_point[3] = 0
        self.suction_point[4] = 0
        self.suction_point[5] = 1
        self.visualize_suction_point()
        ####################################
        self.suction_point = list(self.suction_point)
        e.db(suction_point=self.suction_point)
        e.db(suction_normal=self.suction_point[3:6])

    def call_passive_vision(self, bin_id=0):
        '''Call passive vision, log file paths, and return suction points
        and grasp proposals'''
        param_flush = self.param_flush
        #TODO: Maria isn't doing this
        # Get suction points from vision
        e.db('Explicitly waiting 1 seconds for passive vision to catch up')
        time.sleep(1)
        e.db('Calling passive vision system for suction proposals.',
             bin_id=bin_id, theme='vision')
        getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
        e.db(bin_id=bin_id)
        self.passive_vision_state = getPassiveVisionEstimate('', bin_id)
        e.db('Received suction and grasp proposals.')
        #Suction
        self.all_suction_points = np.asarray(self.passive_vision_state.suction_points)
        self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/7, 7)
        self.visualize_suction_points()
        #Gras
        self.all_grasp_proposals = np.asarray(self.passive_vision_state.grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/12, 12)
        self.visualize_grasping_proposals(self.all_grasp_proposals, False)
        #FlushGrasp
        self.all_flush_proposals = np.asarray(self.passive_vision_state.flush_grasp_proposals)
        self.all_flush_proposals = self.all_flush_proposals.reshape(len(self.all_flush_proposals)/param_flush, param_flush)
        print self.all_flush_proposals.shape
        #self.visualize_flush_proposals()

        # Log saved image files and camera intrinsics files
        if self.experiment:
            self.log_file.write('passive_vision_files {}\n'.format(self.passive_vision_state.file_paths[0]))

        # Penalize recent attempts
        self.adjustPredictionsWithFailedAttempts()
        self.all_suction_points = self.all_suction_points[self.all_suction_points[:,self.param_suction-1].argsort()[::-1]]

    def CallActiveVision(self, bin_id):
        '''Call Active Vision and predict the name of the object. Add log
        @param bin_id Bin to perform vision on
        @return predicted_object_name Guesses name of object'''
        if self.in_simulation:
            return self.callFakeActiveVision(self.tote_ID)
        e.db('Calling active vision system to recognize object in gripper.')
        getActiveVisionRecognition = rospy.ServiceProxy('/active_vision/recognize', active_vision.srv.recognition)
        active_vision_data = getActiveVisionRecognition(bin_id, self.weight_info['weights']/1000.0, self.toteObjNames())
        highest_confidence = 0
        predicted_object_idx = 0
        for i in range(0, len(active_vision_data.object_confidence)):
            if active_vision_data.object_confidence[i] > highest_confidence:
                highest_confidence = active_vision_data.object_confidence[i]
                predicted_object_idx = i
        predicted_object_name = active_vision_data.object_name[predicted_object_idx]
        if self.experiment:
            self.log_file.write('active_vision_files {}\n'.format(active_vision_data.file_paths[0]))
            self.log_file.write('active_vision_prediction {}\n'.format(predicted_object_name))
        return predicted_object_name, active_vision_data.bbox_info

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
            self.objects.append(Item(container=0))
            self.objects[-1].update_with_label(obj)
        self.toteObj = range(len(self.objects))
        for x in range(self.num_bins):
            for obj in DATA['bins'][x]['contents']: 
                self.objects.append(Item(container=x+1)) 
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
        self.data_directory = '/home/mcube/arcdata/loopdata/planner/{}'.format(int(round(time.time())))
        os.makedirs(self.data_directory)
        self.infinite_looping = True

    def json_experiment(self):
        '''Log the experimental data in json files
        @return outfilename Name of file where data is stored'''
        execution_success_string = 'success'
        if not self.execution_result:
            execution_success_string = 'fail'
        self.outfilename = '/home/mcube/arcdata/planner_experiments/%s_%s_%s.json' %(self.execution_date, self.best_primitive, execution_success_string)
        with open(self.outfilename, 'w') as outfile:
            json.dump(self.experiment_data, outfile, sort_keys=True,
                      indent=4, ensure_ascii=False)
        with open(self.outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(self.experiment_data, outfile, sort_keys=True,
                      indent=4, ensure_ascii=False)
        return self.outfilename
    
    def json_output(self):
        outfilename = os.environ['ARC_BASE']+'/output/stow_result.json'
        DATA = {}
        DATA['bins'] = []
        for x in range(self.num_bins):
            DATA['bins'].append({'bin_id': chr(ord('A')+x), 'contents': self.shelfObjNamesGivenBin(x+1) })
        DATA['boxes'] = []
        DATA['tote'] = {}
        DATA['tote']['contents'] = self.toteObjNames()
        with open(outfilename, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(outfilename + '.' + str(rospy.get_time()), 'w') as outfile:   # save every iteration seperately in case the result got overwritten
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)

    
    def GetSuctionPoints(self, number_points):
        ''' Call vision and do some math on it (?) to return maximally
        number_points number of suction points
        @param number_points Number of suction points to return
        @return suction_points Suction points with their scores'''
        param_suction = self.param_suction
        e.db(toteID=self.tote_ID)
        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](self.tote_ID)
        e.db('Recieved suction proposals.', theme='vision')
        #I already do the next 2 lines in passiveVision
        #self.all_suction_points = np.asarray(self.all_suction_points)
        #self.all_suction_points = self.all_suction_points.reshape(len(self.all_suction_points)/param_suction,param_suction)

        # NOTE: REMOVE THIS WHEN SUCTION DIRECTION IS CONTROLLABLE
        # Only keep suction points that are valid for suction down (with surface normals close to gravity)
        updown_suction_points_indices = []
        e.db(SUCTIONPOINTS=self.all_suction_points)
        for i in range(0, self.all_suction_points.shape[0]):
            angle2gravity = np.rad2deg(np.arctan2(la.norm(np.cross([self.all_suction_points[i, 3], self.all_suction_points[i, 4], self.all_suction_points[i, 5]], [0, 0, 1])),
                            np.dot([self.all_suction_points[i, 3], self.all_suction_points[i, 4], self.all_suction_points[i, 5]], [0, 0, 1])))
            if angle2gravity < 40: # If angle to gravity direction is less than 20 degrees
                updown_suction_points_indices.append(i)
        updown_suction_points = np.zeros((len(updown_suction_points_indices), param_suction))
        for i in range(0, len(updown_suction_points_indices)):
            updown_suction_points[i] = self.all_suction_points[updown_suction_points_indices[i], :]
        self.all_suction_points = updown_suction_points

        ## Remove suction points that are repeated:
        not_bad_suction_points_indices = []
        print 'bad_points: ', self.bad_suction_points
        for i in range(0, self.all_suction_points.shape[0]):
            is_bad = False
            suction_point = list(self.all_suction_points[i, 0:param_suction-1])
            for bad_point in self.bad_suction_points:
                if np.linalg.norm(np.array(bad_point[0:2]) - np .array(suction_point[0:2]))< 0.025: #only looking position, not angle
                    is_bad = True
                    e.db('..................................................................................................................')
                    e.db('Removed suction point: ', suction_point[0:2], ' because of previous bad point: ', bad_point[0:2])
                    break
            if not is_bad:
                not_bad_suction_points_indices.append(i)
        not_bad_suction_points = np.zeros((len(not_bad_suction_points_indices), param_suction))
        for i in range(0, len(not_bad_suction_points_indices)):
            not_bad_suction_points[i] = self.all_suction_points[not_bad_suction_points_indices[i], :]
        self.all_suction_points = not_bad_suction_points


        if self.all_suction_points == []:
            return None, None
        #e.db(SUCTIONPOINTS=self.all_suction_points)
        if number_points == 1:
            return self.all_suction_points[0, 0:param_suction-1], self.all_suction_points[0, param_suction-1]
        return self.all_suction_points[:, 0:param_suction-1], self.all_suction_points[:, param_suction-1]

    def GetGraspPoints(self, num_points):
        '''Call passive vision to get grasp points. Sort them by score
        and return maximally num_points number of suction points.
        @param num_points Number of grasp points to return '''
        param_grasping = self.param_grasping
        param_flush = self.param_flush
        e.db(toteID=self.tote_ID)
        if self.all_suction_points is None:
            self.passiveVisionTypes[self.visionType](self.tote_ID)
        e.db('[Vision] Received grasp proposals.')
        #e.db(GraspPointsShape=self.all_grasp_proposals.shape)
        #I already do the next 2 lines in PassiveVision
        num_points = min(num_points, len(self.all_grasp_proposals))
        #e.db(num_points_grasp=num_points)
        #e.db(graspproposals=self.all_grasp_proposals)
        #e.db(flush_proposals=self.all_flush_proposals)
        self.all_pick_proposals = list(self.all_grasp_proposals[0:num_points, 0:param_grasping])
        self.all_pick_scores = list(self.all_grasp_proposals[0:num_points, param_grasping-1])
        # If we have flush proposals, join them to the grasps
        if False: # TODO: len(self.all_flush_proposals) > 0:
            self.all_pick_proposals += list(self.all_flush_proposals[:, 0:param_flush])
            self.all_pick_scores += list(self.all_flush_proposals[:, param_flush-1])

        self.all_pick_scores.sort(reverse=True)
        self.all_pick_proposals.sort(key=lambda x: x[-1], reverse=True)
        self.num_pick_proposals = len(self.all_pick_scores)
        #e.db(pick_proposals=self.all_pick_proposals)
        #e.db(pick_scores=self.all_pick_scores)

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

    def experiment_preparation(self):
        pass
    def aux_vision_logs(self):
        '''Write to vision logs'''
        #################
        ## VISION LOGS ##
        #################
        self.log_file = open(self.data_directory + '/{}.txt'.format(int(round(time.time()))), 'w')
        self.log_file.write('pick source: {}\n'.format(self.tote_ID))
        self.log_file.write('action primitive: {}\n'.format(self.best_primitive))
        self.vision_info = self.data_directory + '/log.txt'

    def runActiveID(self):
        '''Run either real or fake vision to get the ID from active vision'''
        time.sleep(1) #make sure robot stopped
        if self.visionType == 'virtual':
            self.active_vision_ID, self.bbox_info = self.callFakeActiveVision(self.tote_ID)
        else:
            self.active_vision_ID, self.bbox_info = self.CallActiveVision(self.tote_ID) #TODO just because it failsself.CallActiveVision(self.tote_ID)

    def run_suction(self):
        '''Activate sution and log whether successful or not.'''
        self.dirty_bin(self.tote_ID)
        #self.suction_point = [1.0511255264282227, -0.44232940673828125, -0.21663841009140014, 0.0, 0.0, 1.0]
        self.primitive_point = self.suction_point
        self.primitive_score = self.suction_score
        if self.in_simulation:
            self.callFakeSuction(prob=0.5)
            return
        e.db(SuctionPositionTarget=self.suction_point[0:3])
        e.db(SuctionNormal=self.suction_point[3:6])
        self.suction_output = suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause,
                        suction_position_target=self.suction_point[0:3],
                        surface_normal=self.suction_point[3:6], flag=0,
                        bin_id=self.tote_ID)
                        #print_messages=False
        self.primitive_output = self.suction_output
        self.plan_possible = self.suction_output['success_flag'] #suction_output['plan_possible']
        self.suction_check = self.suction_output['suction_check']
        #Go home
        suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=1)
        self.execution_possible = (self.plan_possible and self.suction_check)
        e.db(plan_possible=self.plan_possible)
        e.db(suction_check=self.suction_check)
        self.clean_bin(self.tote_ID)

    def run_grasping(self):
        '''Activate either regular or flush grasping. Log whether the
        execution is successful or not'''
        self.dirty_bin(self.tote_ID)
        self.primitive_score = self.grasp_score
        self.primitive_point = self.grasp_point
        if self.in_simulation:
            e.db('CALLING VIRTUAL GRASPING')
            self.callFakeGrasping(prob=0.5)
            return
        #e.db(grasp_point=self.grasp_point)
        #e.db(LENGTH=len(self.grasp_point))
        #e.db(PARAMFLUSH=self.param_flush)
        if len(self.grasp_point) == self.param_flush:
            self.selected_grasp_type = 'flush'
            self.grasping_output = flush_grasp(objInput=self.grasp_point,
                                               listener=self.listener,
                                               br=self.br,
                                               isExecute=self.isExecute,
                                               binId=self.tote_ID, flag=0,
                                               withPause=self.withPause)
        else:
            self.selected_grasp_type = 'normal'
            self.grasping_output = grasp(objInput=self.grasp_point,
                                         listener=self.listener,
                                         br=self.br,
                                         isExecute=self.isExecute,
                                         binId=self.tote_ID,
                                         flag=0,
                                         withPause=self.withPause)
        self.pick_possible = self.grasping_output['grasp_possible']
        self.plan_possible = self.grasping_output['plan_possible']
        self.execution_possible = self.grasping_output['execution_possible']
        self.collision_detected = self.grasping_output['collision']
        self.grasping_output['graspPose'] = list(self.grasping_output['graspPose'])
        self.primitive_output = self.grasping_output
        e.db(pick_possible=self.pick_possible)
        e.db(plan_possible=self.plan_possible)
        e.db(execution_possible=self.execution_possible)
        self.clean_bin(self.tote_ID)

    def getBestGraspingPoint(self):
        '''Loop through all possible grasping points starting with highest
        score. Skip if in virtual. Otherwise, execute either a flush or
        regular pick. '''
        #number_grasp_points = 1
        self.GetGraspPoints(num_points=20)
        #e.db(pick_proposals=self.all_pick_proposals)
        #e.db(pick_scores=self.all_pick_scores)
        e.db(num_pick_proposals=self.num_pick_proposals)
        num_atempts = 0
        self.execution_possible = False
        while not self.execution_possible and num_atempts < self.num_pick_proposals:
            e.db(num_atempts)
            self.grasp_point = list(self.all_pick_proposals[num_atempts][:])
            
            self.visualize_grasping_proposals(np.array([self.grasp_point]), True)

            self.grasp_score = self.all_pick_scores[num_atempts]
            num_atempts += 1

            e.db(grasp_point=self.grasp_point)
            if self.visionType == 'virtual': #assume point is good if in virtual
                return
            #e.db(LENGTH=len(self.grasp_point))
            #e.db(PARAM_FLUSH=self.param_flush)
            if len(self.grasp_point) == self.param_flush:
                self.selected_grasp_type = 'flush'
                checked_output = flush_grasp(objInput=self.grasp_point,
                                             listener=self.listener,
                                             br=self.br,
                                             isExecute=False,
                                             binId=self.tote_ID, flag=0,
                                             withPause=False)
            else:
                self.selected_grasp_type = 'normal'
                checked_output = grasp(objInput=self.grasp_point,
                                       listener=self.listener,
                                       br=self.br,
                                       isExecute=False,
                                       binId=self.tote_ID, flag=0,
                                       withPause=False)
            if checked_output['execution_possible']:
                return
            self.execution_possible = checked_output['execution_possible']
            self.collision_detected = checked_output['collision']
            #self.logGraspOutputForVision()  #TODO: why was this here?
        e.db('NONE OF THE GRASPING POINTS WORK')
        self.grasp_point = None
        self.grasp_score = 0
        return

    def logGraspOutputForVision(self):
        '''Log the output of the grasping parameters'''
        self.log_file.write('selected_grasp_type {}\n'.format(self.selected_grasp_type))
        self.log_file.write('selected_grasp_proposal: {}\n'.format(self.grasp_point))
        #self.log_file.write('selected_grasp_proposal {}\n'.format(selected_grasp_proposal.tolist()))
        self.log_file.write('collision_detected: {}\n'.format(self.collision_detected))
        self.log_file.write('object_in_grasp: {}\n'.format(self.execution_possible))

    def experiment_pick(self):
        '''Execute either a suction or grasp primitive based on what vision
        has determined to be the best option '''
        self.best_primitive = self.primitives[0]
        self.aux_vision_logs()
        if self.best_primitive == 'suction-tote':
            self.getBestSuctionPoint()
            self.run_suction()
            self.log_file.write('selected_suction_point: {}\n'.format(self.suction_point))
            self.log_file.write('suction_success: {}\n'.format(self.suction_check))
        elif self.best_primitive == 'grasp-tote':
            self.getBestGraspingPoint()
            self.primitive_point = self.grasp_point
            if self.grasp_point is not None:
                self.run_grasping()
                self.logGraspOutputForVision()
            #Go home
            grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                  binId=self.tote_ID, flag=1, withPause=self.withPause)
        else:
            assert False, 'Either suction-tote or grasp-tote'

    def planned_place(self):
        '''Call Ferran's placing planner based on height. Compute height map and
        either place via suction or drop in a specific location (from planner)'''
        self.dirty_bin(self.objects[self.placedObj].container)
        self.objects[self.placedObj].conservative_w_dim_from_dim()
        #self.objects[self.placedObj].w_dim = self.objects[self.placedObj].dim #TODO: get from vision
        self.PlacingPlanner.place_object_local_best(self.placedObj)
        self.PlacingPlanner.update_pose_placed_obj(self.placedObj)
        e.db('Placing on bin ', self.objects[self.placedObj].container, ' position ', self.objects[self.placedObj].pos, 'pose' , self.objects[self.placedObj].pose)
        drop_pose = self.objects[self.placedObj].pose
        drop_pose[2] += rospy.get_param("/tote/height")
        e.db(drop_pose)
        if self.best_primitive == 'suction-tote':
            suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause, flag=2,
                                bin_id=self.objects[self.placedObj].container,
                                suction_position_target = drop_pose) 
        else:
            grasping_output_drop=grasp(objInput=self.grasp_point,
                                       listener=self.listener,
                                       br=self.br,
                                       isExecute=self.isExecute,
                                       objId=self.pickedLabel,
                                       binId=self.objects[self.placedObj].container,
                                       flag=2,
                                       withPause=self.withPause,
                                       place_pose=drop_pose)
        self.clean_bin(self.objects[self.placedObj].container)

    def default_place(self):
        '''Default placing planner which simply using suction or grasping to
        drop in the middle of the bin.'''
        self.drop_id = 1 - self.tote_ID
        e.db(drop_id=self.drop_id)
        self.dirty_bin(self.tote_ID)
        if self.best_primitive == 'suction-tote':
            # Drop object over destination (random location, fixed z)
            suction_down_simple(listener=self.listener, br=self.br, withPause=self.withPause,
                                flag=2, bin_id=self.drop_id)                              
        else: #assuming grasping
            grasping_output_drop = grasp(objInput=self.grasp_point,
                                         listener=self.listener,
                                         br=self.br,
                                         isExecute=self.isExecute,
                                         objId=self.pickedLabel,
                                         binId=self.drop_id, flag=2,
                                         withPause=self.withPause)
        self.clean_bin(self.tote_ID)

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
                               #'execution_possible': self.execution_possible,
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
                                'weight_info': self.weight_info,
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

    def switch_tote(self):
        '''Swap the ids of the shelf and tote'''
        e.db('CHANGING TOTE!')
        self.tote_ID = 3-self.tote_ID #TODO: make it more general
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
        markers_msg.markers.append(createDeleteAllMarker('grap_points'))
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
        pauseFunc(True)
        
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
        pauseFunc(True)
        
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
        
        # proposals  n*12 2d list
        #proposals = np.array([[0.9897886514663696, -0.5969775915145874, -0.1092456579208374, 0.0, 0.0, -1.0, 0.1107543408870697, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.9370370507240295, 0.982134997844696, -0.5985000133514404, -0.1092456579208374, 0.0, 0.0, -1.0, 0.1107543408870697, 0.10999999940395355, 0.0, 1.0, 0.0, 0.936296284198761, 1.0021350383758545, -0.5985000133514404, -0.10864522308111191, 0.0, 0.0, -1.0, 0.1113547757267952, 0.10999999940395355, 0.0, 1.0, 0.0, 0.9125925898551941, 0.969788670539856, -0.5969775915145874, -0.1090724840760231, 0.0, 0.0, -1.0, 0.110927514731884, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.9088888764381409, 0.9621350169181824, -0.5985000133514404, -0.1090724840760231, 0.0, 0.0, -1.0, 0.110927514731884, 0.10999999940395355, 0.0, 1.0, 0.0, 0.9081481695175171, 1.029788613319397, -0.5600224137306213, -0.10899502038955688, 0.0, 0.0, -1.0, 0.11100497841835022, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.8785185217857361, 1.0221350193023682, -0.5985000133514404, -0.10899502038955688, 0.0, 0.0, -1.0, 0.11100497841835022, 0.10999999940395355, 0.0, 1.0, 0.0, 0.8762962818145752, 0.9897886514663696, -0.5600224137306213, -0.1092456579208374, 0.0, 0.0, -1.0, 0.1107543408870697, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.8757575750350952, 1.0162771940231323, -0.5926421284675598, -0.10864522308111191, 0.0, 0.0, -1.0, 0.1113547757267952, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.8703030347824097, 1.0097886323928833, -0.5600224137306213, -0.10864522308111191, 0.0, 0.0, -1.0, 0.1113547757267952, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.8660606145858765, 0.9497886896133423, -0.5969775915145874, -0.10871227085590363, 0.0, 0.0, -1.0, 0.11128773540258408, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.8600000143051147, 1.036277174949646, -0.5726421475410461, -0.10890824347734451, 0.0, 0.0, -1.0, 0.11109175533056259, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.8570370078086853, 1.029788613319397, -0.5769776105880737, -0.10890824347734451, 0.0, 0.0, -1.0, 0.11109175533056259, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.8562963008880615, 0.9962771534919739, -0.5926421284675598, -0.1092456579208374, 0.0, 0.0, -1.0, 0.1107543408870697, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.8527272939682007, 0.9421349763870239, -0.5985000133514404, -0.10871227085590363, 0.0, 0.0, -1.0, 0.11128773540258408, 0.10999999940395355, 0.0, 1.0, 0.0, 0.8481481671333313, 0.9762771129608154, -0.5926421284675598, -0.1090724840760231, 0.0, 0.0, -1.0, 0.110927514731884, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.846666693687439, 0.969788670539856, -0.5600224137306213, -0.1090724840760231, 0.0, 0.0, -1.0, 0.110927514731884, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.8399999737739563, 1.0162771940231323, -0.5726421475410461, -0.10926124453544617, 0.0, 0.0, -1.0, 0.11073875427246094, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.8206060528755188, 0.9497886896133423, -0.5600224137306213, -0.10871227085590363, 0.0, 0.0, -1.0, 0.11128773540258408, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.817037045955658, 1.0097886323928833, -0.5969775915145874, -0.10864522308111191, 0.0, 0.0, -1.0, 0.1113547757267952, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.7993939518928528, 1.0497887134552002, -0.5600224137306213, -0.10972268134355545, 0.0, 0.0, -1.0, 0.11027731746435165, 0.10000000149011612, -0.3826834261417389, -0.9238795042037964, 0.0, 0.7933333516120911, 1.029788613319397, -0.5969775915145874, -0.10899502038955688, 0.0, 0.0, -1.0, 0.11100497841835022, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.7830303311347961, 1.0097886323928833, -0.5769776105880737, -0.10926124453544617, 0.0, 0.0, -1.0, 0.11073875427246094, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.78242427110672, 1.0421350002288818, -0.5985000133514404, -0.10972268134355545, 0.0, 0.0, -1.0, 0.11027731746435165, 0.10000000149011612, 0.0, 1.0, 0.0, 0.7704761624336243, 0.9497886896133423, -0.6169775724411011, -0.11509916931390762, 0.0, 0.0, -1.0, 0.10490082949399948, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.7666666507720947, 1.0021350383758545, -0.578499972820282, -0.10926124453544617, 0.0, 0.0, -1.0, 0.11073875427246094, 0.10999999940395355, 0.0, 1.0, 0.0, 0.7654545307159424, 0.9562771320343018, -0.6126421093940735, -0.11509916931390762, 0.0, 0.0, -1.0, 0.10490082949399948, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.7644444704055786, 0.9562771320343018, -0.5926421284675598, -0.10871227085590363, 0.0, 0.0, -1.0, 0.11128773540258408, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.7551515102386475, 0.9762771129608154, -0.6126421093940735, -0.1090744212269783, 0.0, 0.0, -1.0, 0.1109255775809288, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.7539393901824951, 0.982134997844696, -0.578499972820282, -0.10993495583534241, 0.0, 0.0, -1.0, 0.1100650429725647, 0.10999999940395355, 0.0, 1.0, 0.0, 0.7509090900421143, 0.9897886514663696, -0.5769776105880737, -0.10993495583534241, 0.0, 0.0, -1.0, 0.1100650429725647, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.7484848499298096, 1.0221350193023682, -0.578499972820282, -0.10890824347734451, 0.0, 0.0, -1.0, 0.11109175533056259, 0.10999999940395355, 0.0, 1.0, 0.0, 0.7478787899017334, 0.969788670539856, -0.6169775724411011, -0.1090744212269783, 0.0, 0.0, -1.0, 0.1109255775809288, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.7248485088348389, 0.9621350169181824, -0.6184999942779541, -0.1090744212269783, 0.0, 0.0, -1.0, 0.1109255775809288, 0.10999999940395355, 0.0, 1.0, 0.0, 0.7212121486663818, 0.9421349763870239, -0.6184999942779541, -0.11509916931390762, 0.0, 0.0, -1.0, 0.10490082949399948, 0.10999999940395355, 0.0, 1.0, 0.0, 0.7127272486686707, 0.9221349954605103, -0.5985000133514404, -0.11851752549409866, 0.0, 0.0, -1.0, 0.10148248076438904, 0.10000000149011612, 0.0, 1.0, 0.0, 0.6780952215194702, 1.036277174949646, -0.5843578577041626, -0.10831061750650406, 0.0, 0.0, -1.0, 0.11168938130140305, 0.10999999940395355, -0.7071067690849304, -0.7071067690849304, 0.0, 0.6775757670402527, 1.0497887134552002, -0.5969775915145874, -0.10972268134355545, 0.0, 0.0, -1.0, 0.11027731746435165, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.6678788065910339, 1.0497887134552002, -0.5769776105880737, -0.11163860559463501, 0.0, 0.0, -1.0, 0.1083613932132721, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.6629629731178284, 0.9297886490821838, -0.5600224137306213, -0.11851752549409866, 0.0, 0.0, -1.0, 0.10148248076438904, 0.10000000149011612, -0.3826834261417389, -0.9238795042037964, 0.0, 0.6609523892402649, 1.029788613319397, -0.580022394657135, -0.10831061750650406, 0.0, 0.0, -1.0, 0.11168938130140305, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.657575786113739, 1.0421350002288818, -0.578499972820282, -0.11163860559463501, 0.0, 0.0, -1.0, 0.1083613932132721, 0.10999999940395355, 0.0, 1.0, 0.0, 0.6466666460037231, 1.0562771558761597, -0.5843578577041626, -0.10883389413356781, 0.0, 0.0, -1.0, 0.1111661046743393, 0.10000000149011612, -0.7071067690849304, -0.7071067690849304, 0.0, 0.6390476226806641, 1.0497887134552002, -0.5400224328041077, -0.11163860559463501, 0.0, 0.0, -1.0, 0.1083613932132721, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.6351515054702759, 0.9421349763870239, -0.578499972820282, -0.11022872477769852, 0.0, 0.0, -1.0, 0.10977127403020859, 0.10999999940395355, 0.0, 1.0, 0.0, 0.6193939447402954, 0.9562771320343018, -0.5443578362464905, -0.11022872477769852, 0.0, 0.0, -1.0, 0.10977127403020859, 0.10999999940395355, -0.7071067690849304, -0.7071067690849304, 0.0, 0.6193939447402954, 1.0206125974655151, -0.5461536645889282, -0.10967990010976791, 0.0, 0.0, -1.0, 0.11032009869813919, 0.10999999940395355, -0.9238795042037964, 0.3826834261417389, 0.0, 0.6193939447402954, 0.9297886490821838, -0.6169775724411011, -0.12016915529966354, 0.0, 0.0, -1.0, 0.09983084350824356, 0.10000000149011612, -0.3826834261417389, 0.9238795042037964, 0.0, 0.616190493106842, 0.9497886896133423, -0.5400224328041077, -0.11022872477769852, 0.0, 0.0, -1.0, 0.10977127403020859, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.6139394044876099, 1.0406125783920288, -0.5461536645889282, -0.11048819869756699, 0.0, 0.0, -1.0, 0.10951180011034012, 0.10999999940395355, -0.9238795042037964, 0.3826834261417389, 0.0, 0.6111111044883728, 0.9297886490821838, -0.5969775915145874, -0.11851752549409866, 0.0, 0.0, -1.0, 0.10148248076438904, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.6051852107048035, 1.0562771558761597, -0.5726421475410461, -0.11163860559463501, 0.0, 0.0, -1.0, 0.1083613932132721, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.5866666436195374, 0.980612576007843, -0.6261536478996277, -0.1083928719162941, 0.0, 0.0, -1.0, 0.111607126891613, 0.10999999940395355, -0.9238795042037964, 0.3826834261417389, 0.0, 0.5718518495559692, 1.0421350002288818, -0.6184999942779541, -0.10883389413356781, 0.0, 0.0, -1.0, 0.1111661046743393, 0.10999999940395355, 0.0, 1.0, 0.0, 0.5672727227210999, 0.9221349954605103, -0.6184999942779541, -0.12016915529966354, 0.0, 0.0, -1.0, 0.09983084350824356, 0.10999999940395355, 0.0, 1.0, 0.0, 0.5644444227218628, 1.0497887134552002, -0.580022394657135, -0.10883389413356781, 0.0, 0.0, -1.0, 0.1111661046743393, 0.10999999940395355, -0.3826834261417389, -0.9238795042037964, 0.0, 0.5614814758300781, 0.9362771511077881, -0.6126421093940735, -0.12016915529966354, 0.0, 0.0, -1.0, 0.09983084350824356, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.5533333420753479, 1.036277174949646, -0.5526421070098877, -0.11048819869756699, 0.0, 0.0, -1.0, 0.10951180011034012, 0.10999999940395355, -0.7071067690849304, 0.7071067690849304, 0.0, 0.5478788018226624, 0.9297886490821838, -0.5400224328041077, -0.11865722388029099, 0.0, 0.0, -1.0, 0.10134277492761612, 0.10000000149011612, -0.3826834261417389, -0.9238795042037964, 0.0, 0.5104761719703674, 1.0562771558761597, -0.5526421070098877, -0.11621406674385071, 0.0, 0.0, -1.0, 0.1037859320640564, 0.10000000149011612, -0.7071067690849304, 0.7071067690849304, 0.0, 0.49047619104385376, 0.9362771511077881, -0.6326421499252319, -0.13637368381023407, 0.0, 0.0, -1.0, 0.08362631499767303, 0.07999999821186066, -0.7071067690849304, 0.7071067690849304, 0.0, 0.4546666741371155, 0.9362771511077881, -0.5443578362464905, -0.11865722388029099, 0.0, 0.0, -1.0, 0.10134277492761612, 0.10999999940395355, -0.7071067690849304, -0.7071067690849304, 0.0, 0.4333333373069763, 0.9221349954605103, -0.578499972820282, -0.11865722388029099, 0.0, 0.0, -1.0, 0.10134277492761612, 0.10999999940395355, 0.0, 1.0, 0.0, 0.432121217250824, 1.0497887134552002, -0.5569775700569153, -0.11621406674385071, 0.0, 0.0, -1.0, 0.1037859320640564, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.39878788590431213, 1.0606125593185425, -0.5461536645889282, -0.11621406674385071, 0.0, 0.0, -1.0, 0.1037859320640564, 0.10999999940395355, -0.9238795042037964, 0.3826834261417389, 0.0, 0.3884848356246948, 0.9406126141548157, -0.6261536478996277, -0.13637368381023407, 0.0, 0.0, -1.0, 0.08362631499767303, 0.10000000149011612, -0.9238795042037964, 0.3826834261417389, 0.0, 0.3733333349227905, 0.9297886490821838, -0.6369776129722595, -0.13637368381023407, 0.0, 0.0, -1.0, 0.08362631499767303, 0.10999999940395355, -0.3826834261417389, 0.9238795042037964, 0.0, 0.3224242329597473]])
        #proposals = self.all_pick_proposals.reshape((self.all_pick_proposals.shape[1]/12,12))
        # hack above
        
        n = proposals.shape[0]
        
        color = (0, 1, 1, 1)  # rgba
        if is_selected:
            color = (1, 1, 0, 1)
        color_bar = (0.5, 0.5, 0.5, 0.5)  # rgba
        scale = (0.001,0.02,0.1)
        markers_msg = MarkerArray()
        #if not is_selected:  # don't delete other candidate
            #markers_msg.markers.append(createDeleteAllMarker('pick_proposals'))
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
        #rospy.sleep(0.1)
        pauseFunc(True)
        
    ######################
    ### MAIN FUNCTIONS ###
    ######################
    def run_stowing(self):
        '''Main experimental loop. This will loop until all objects are cleared
        or the time limit is hit. Generically calls each action, with the mode
        being indicated by the self.decider. Stows objects and logs all data'''
        self.time_begin = time.time()
        self.time_end = self.time_begin + self.duration_of_contest
        #Initialize robot state
        #TODO
        #gripper.close()
        goToHome.goToARC(slowDown=self.goHomeSlow)
        self.preparation[self.decider]()
        self.json_output()
        while len(self.toteObj) > 0 and time.time() < self.time_end:
            self.initial_seq_time = time.time()
            self.all_suction_points = None
            if not self.in_simulation:                
                #Calibrate weigth sensor
                self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)
            self.weight_info = self.weightSensor.readWeightSensor(item_list=self.toteObjNames(), 
                                             withSensor=self.withSensorWeight, binNum=self.tote_ID) 
            e.db('Initial weight (should be zero)',  self.weight_info['weights'])
            #TODO: update visualization
            self.primitive_score = 0.0
            self.primitive_output = {}
            self.pick[self.decider]()
            if not self.in_simulation:
                self.weight_info = self.weightSensor.readWeightSensor(item_list=self.toteObjNames(),
                        withSensor=self.withSensorWeight, binNum=self.tote_ID)
            e.db('detected weight',  self.weight_info['weights'])
            
            if self.weight_info['weights'] > 100: #TODO: minimum object weight?
                self.execution_possible = True
            
            if not self.execution_possible:
                suction.stop()
            
            # Query the user to determine if the experiment succeeded
            if self.human_supervision:
                self.execution_possible = self.GetHumanInputOnExperimentStatus()

            if self.execution_possible or self.forceSucc:
                e.db('Running active ID')
                try:
                    self.runActiveID()
                    self.pickedLabel = self.active_vision_ID
                except:
                    e.db('PROBLEM: ACTIVE ID CRASHED!!!!')
                    import traceback
                    traceback.print_ex
                    self.pickedLabel = 'unknown'
                if self.pickedLabel not in self.toteObjNames():
                    e.db("PROBLEM: Active ID gave us ID not in tote objects, assuming it's ", self.toteObjNames()[0])
                    self.pickedLabel = self.toteObjNames()[0] #TODO: are we happy with this?
            e.db('Object picked acording active vision: ', self.pickedLabel)
            if self.human_supervision:
                e.db('ID given by active vision: ', self.active_vision_ID)
                self.pickedLabel = self.GetHumanToLabelObject(self.execution_possible)
            
            if self.experiment:
                self.log_file.write('pick_successful: {}\n\n\n'.format(self.execution_possible))

            self.json_output()
            if (self.execution_possible or self.forceSucc): #success
                self.fails_in_row = 0

                #TODO: if baseline, by now we want to consider that picked object is the one with higher probability
                '''
                if not self.in_simulation and not self.experiment:
                    x = self.weight_info['probs']
                    self.pickedLabel = self.objects[x.index(max(x))].label
                '''
                ## Check object in toteObj, remove it and update json
                for i, x in enumerate(self.toteObj):
                    if self.objects[x].label == self.pickedLabel:
                        self.placedObj = i
                        self.shelfObj.append(x)
                        self.objects[x].swap_container()
                        self.toteObj.pop(i)
                        e.db('Object placed: ', self.pickedLabel)
                        break
                    ''' TODO: look at this
                    else:
                        #FIXME this should never happen because we only move objects with known labels
                        assert self.pickedLabel == 'unknown', 'Object not found: '+str(self.toteObjNames())+' '+str(self.pickedLabel)
                    '''
                self.placer[self.decider]()
            else:
                # Suction or Grasping Failed. Log it.
                if self.best_primitive == 'suction-tote':
                    subprocess.call(['rosservice', 'call', '/suction_service', '"soff"', '""'])
                    self.bad_suction_points.append(self.primitive_point)
                    self.bad_suction_times.append(time.time())
                else: #assuming grasping
                    # We only want to append lists to grasping points
                    if type(self.primitive_point) is list:
                        self.bad_grasping_points.append(self.primitive_point)
                        self.bad_grasping_times.append(time.time())
                    # If no grasping points are found, self.primitive_point is None.
                    # We don't want to include this in the list. However, any other
                    # Python type probably means unexpected (buggy) behavior
                    elif self.primitive_point is not None:
                        raise TypeError('Primitive Points for grasping are of \
                               type {}, insted of list or none.'.format(type(self.primitive_point)))
                self.fails_in_row += 1
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

        # To Finish planning, move to set location and open the gripper
        goToHome.goToARC(slowDown = self.goHomeSlow)
        gripper.grasp_in(10, 3)
        e.db("Planner is done")

    def run(self):
        '''Exeecutes only the stowing task'''
        if self.task == 'stowing':
            self.run_stowing()
        else:
            assert False, 'I can only do stowing'

if __name__ == '__main__':
    ## Explainer for outputs
    e = Explainer()
    e.set_current_theme('planner')
    ## Parse arguments
    parser = optparse.OptionParser()

    parser.add_option('-v', '--vision', action='store', dest='visionType',
        help='real or virtual', default='real')

    parser.add_option('-p', '--pause', action='store_true', dest='withPause',
        help='To pause or not', default=False)

    parser.add_option('-n', '--noexe', action='store_false', dest='isExecute',
        help='To execute or not', default=True)

    parser.add_option('-f', '--forcesucc', action='store_true', dest='forceSucc',
        help='To force success on trying the last strategy in the strategy list; This is useful for testing system in virtual environment.',
        default=False)

    parser.add_option('-j', '--jsonfilename', action='store', dest='jsonfilename',
        help='Name of the json file', default='apc_stow_task17.json')

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
        help='Task to run [right now only stowing]')

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

    p = TaskPlanner(args)
    p.call_passive_vision()
    p.getBestSuctionPoint()
    p.getBestGraspingPoint()
