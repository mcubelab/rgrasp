#!/usr/bin/python


from placing_planner17 import PlacingPlanner
import random, subprocess, time, datetime, json, optparse, rospy, copy
import tf
from ik.visualize_helper import visualize_grasping_proposals
import numpy as np
from manual_fit.srv import *
import os
#from collision_detection.collisionHelper import collisionCheck
from sensor_msgs.msg import Image as RosImage

try:
    import passive_vision.srv
    import realsense_camera.srv
except:
    print 'FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL'

import sys
sys.path.append(os.environ['CODE_BASE']+'/catkin_ws/src/weight_sensor/src')
import ws_prob
import sensor_msgs.msg
import goToHome
from grasping17 import grasp

from ik.roshelper import poseTransform, lookupTransform, lookupTransformList, coordinateFrameTransform, coordinateFrameTransformList, visualizeObjects, pose2list, pubFrame, ROS_Wait_For_Msg
from ik.helper import get_obj_vol, rotmatZ, get_obj_dim, matrix_from_xyzquat, pauseFunc, getObjCOM, quat_from_matrix, in_or_out, moveGripper, fake_bbox_info, fake_bbox_info_1, Timer
from visualization_msgs.msg import MarkerArray, Marker
from apc.helper import UpdateCommand


class TaskPlanner(object):
    def __init__(self, args):
        
        
        self.passiveVisionTypes = {'real' : self.call_passive_vision,
                                   'file' : self.call_passive_vision,
                                   'virtual' : self.callFakePassiveVision}
        # Class constants
        self.param_grasping = 12
        self.infinite_looping = False
        self.max_dimension = 0.1  #TODO M: adjust for realistic bbox size
        self.goHomeSlow = False
        self.tote_ID = 0 
        self.fails_in_row = 0       
        # Configuration
        self.withPause = opt.withPause
        self.experiment = opt.experiment
        self.isExecute = opt.isExecute
        # ROS setup
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #Vision
        self.visionType = opt.visionType
        if self.visionType == 'real':
            self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
        self.FAKE_PASSIVE_VISION_DIR = os.environ['CODE_BASE'] + '/input/fake_dirs/fake_passive_vision/'
        self.FAKE_GRASPING_DIR = os.environ['CODE_BASE'] + '/input/fake_dirs/fake_grasping/'
        self.passive_vision_file_id = opt.passive_vision_file_id
        self.all_grasp_proposals = None
        #Bad points
        self.bad_grasping_points = []
        self.bad_grasping_times = []
        #Placing
        self.PlacingPlanner = PlacingPlanner(visionType = self.visionType)
        #Weights
        self.weightSensor = ws_prob.WeightSensor()
        self.withSensorWeight = True
        if self.visionType != 'real':
            self.withSensorWeight = False
        self.weight_info = [None for _ in range(9)]  #Adding the boxes
        #Class Publishers 
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.proposal_viz_array_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
        self.hm_pre_pub = rospy.Publisher('/height_map_pre', RosImage, queue_size = 10)
        self.hm_post_pub = rospy.Publisher('/height_map_post', RosImage, queue_size = 10)
        self.score_pub = rospy.Publisher('/placing_score', RosImage, queue_size = 10)
        rospy.sleep(0.5)

    ###############################
    ### GLOBAL FUNCTIONS ###
    ###############################

    def request_passive_vision_wait(self, bin_id):
        while True:
            try:
                return self.getPassiveVisionEstimate('request', '', bin_id)
            except:
                print('Waiting Vision.')

    def switch_tote(self): 
        self.tote_ID = 1-self.tote_ID
        self.fails_in_row = 0

    def remove_old_points(self, points, times, limit):
        new_points = []
        new_times = []
        for j, point in enumerate(points):
            if time.time() - times[j] < limit: #self.bad_suction_times[j] < time.time() + 60*3: #We only care about things that happened in the last 3 minutes
                new_points.append(point)
                new_times.append(times[j])
        return new_points, new_times

    ##############################
    ### FAKE VIRTUAL FUNCTIONS ###
    ##############################

    def callFakePassiveVision(self, container):
        print('File used for fake passive vision: ', self.passive_vision_file_id+ '/passive_vision_data_bin_' + str(container) + '.json')
        full_path = self.FAKE_PASSIVE_VISION_DIR + self.passive_vision_file_id
        with open(full_path + '/passive_vision_data_bin_' + '0' + '.json', 'r') as infile:
            DATA = json.load(infile)
        self.all_grasp_proposals = [float(i) for i in DATA['grasp_proposals']]
        self.all_grasp_proposals = np.array(self.all_grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/12, 12)

    def callFakeGrasping(self, prob, container):
        print('------- DOING GRASPING ------- ')
        print(' grasp_point = ', self.grasp_point)
        _ = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
                  binId=container, flag=0, withPause=False, viz_pub=self.viz_array_pub)
        
        f = random.choice(os.listdir(self.FAKE_GRASPING_DIR)) #Get fake output for the primitive
        with open(os.path.join(self.FAKE_GRASPING_DIR, f), 'r') as infile:
            self.grasping_output = json.load(infile)
        self.grasping_output = self.grasping_output['primitive_output']
        #Random decides if execution possible
        self.grasping_output['execution_possible'] = (np.random.rand() <= prob)
        self.execution_possible = self.grasping_output['execution_possible']

    ########################
    ### VISION FUNCTIONS ###
    ########################

    def call_passive_vision(self, bin_id=0):
        print('Calling passive vision system for grasp proposals. The bin considered is: ', bin_id)
        with Timer('getPassiveVisionEstimate ' + 'request hm sg %d' % bin_id ):
            self.passive_vision_state = self.request_passive_vision_wait(bin_id)

        print('Received grasp proposals.')
        self.all_grasp_proposals = np.asarray(self.passive_vision_state.grasp_proposals)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/self.param_grasping, self.param_grasping)
        self.grasp_object_list = np.asarray(self.passive_vision_state.grasp_object_list)
        self.grasp_object_confidence = np.asarray(self.passive_vision_state.grasp_object_confidence)
        #visualize_grasping_proposals(self.proposal_viz_array_pub, self.all_grasp_proposals, False)
        
        #Sorting all points
        grasp_permutation = self.all_grasp_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]
        self.grasp_object_list = self.grasp_object_list[grasp_permutation]
        self.grasp_object_confidence = self.grasp_object_confidence[grasp_permutation]
    
    def GetGraspPoints(self, num_points, container):
        if self.all_grasp_proposals is None:
            self.passiveVisionTypes[self.visionType](container)
        if self.visionType == 'virtual':
            self.grasp_object_list = ['Null']*len(self.all_grasp_proposals)
            self.grasp_object_confidence = ['Null']*len(self.all_grasp_proposals)
        #Add grasp proposals if possible
        print('There are ', len(self.bad_grasping_points), ' bad_grasping_points ',': ', self.bad_grasping_points)

        ## Filter out outdated bad_grasping_point
        self.bad_grasping_points, self.bad_grasping_times = self.remove_old_points(self.bad_grasping_points, self.bad_grasping_times, 60*3)
        if len(self.all_grasp_proposals) > 0:            
            not_bad_grasp_points_indices = [] ## Remove grasp points that are repeated:
            for i in range(0, self.all_grasp_proposals.shape[0]):
                is_bad = False
                grasp_point = list(self.all_grasp_proposals[i, 0:self.param_grasping-1])
                for j,bad_point in enumerate(self.bad_grasping_points):
                    if len(bad_point) == self.param_grasping and  np.linalg.norm(np.array(bad_point[0:2]) - np .array(grasp_point[0:2]))< 0.03: #only looking position, not angle
                        is_bad = True
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
        
        self.all_pick_scores.sort(reverse=True)
        self.all_pick_proposals.sort(key=lambda x: x[-1], reverse=True)
        self.num_pick_proposals = len(self.all_pick_scores)

    def getBestGraspingPoint(self, container):
        self.GetGraspPoints(num_points=100,container = container)
        
        self.execution_possible = False 
        if self.num_pick_proposals == 0: #If no grasp point available
            self.grasp_point = None
            self.grasp_score = 0
            return

        #Find the best point that it is not in collision:
        num_attempts = 0
        num_it = 0
        while not self.execution_possible and num_attempts < 20 and num_it < self.num_pick_proposals: #Each time try at most 20 attempts
            grasp_point = copy.deepcopy(list(self.all_pick_proposals[num_it][:]))
            num_it += 1
            #If we already know it is a bad point, we do not try it again
            if grasp_point in self.bad_grasping_points:
                return
            #Check if in collision
            num_attempts += 1        
            checked_output = grasp(objInput=grasp_point, listener=self.listener, br=self.br, isExecute=False,
                                   binId=container, flag=0, withPause=False, viz_pub=self.viz_array_pub)
            if checked_output['execution_possible']:
                self.grasp_score = copy.deepcopy(self.all_pick_scores[num_it-1])
                self.grasp_point = copy.deepcopy(grasp_point)
                print('Best grasp point:', grasp_point, ' with score: ', self.grasp_score, 'in bin: ', container)
                return
            if grasp_point is not None:
                self.bad_grasping_points.append(grasp_point)
                self.bad_grasping_times.append(-1)
        print('NONE OF THE GRASPING POINTS WORKS')
        self.grasp_point = None
        self.grasp_score = 0
        print('Best grasp point:', grasp_point, ' with score: ', self.grasp_score, 'in bin: ', container)
        return

    ###################
    ### EXPERIMENTS ###
    ###################

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

    def experiment_logging(self):
        '''Create json file of the experiment'''
        self.execution_time = time.time() - self.initial_seq_time
        self.execution_date = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.execution_result = self.execution_possible
        self.experiment_data = {'execution_result' : self.execution_result,
                                'obj_pose': [0]*7,
                                'primitive_point': self.grasp_point,
                                'primitive_score': self.grasp_score,
                                'weight_info': self.weight_info[self.tote_ID],
                                'previous_experiment_name': self.previous_experiment_name,
                                'tote_ID': self.tote_ID,
                                'bad_grasping_points': self.bad_grasping_points,
                                'next_experiment_name' : 'NULL'}
        self.new_experiment_name = self.json_experiment()
        print('This file was stored in ', self.new_experiment_name)
        #Add the 'new experiment' to the previous one
        if self.previous_experiment_name is not None:
            with open(self.previous_experiment_name) as data_file:
                data = json.load(data_file)
                data['next_experiment_name'] = self.new_experiment_name
            with open(self.previous_experiment_name, 'w') as outfile:
                json.dump(data, outfile, sort_keys=True, indent=4, ensure_ascii=False)
        self.previous_experiment_name = self.new_experiment_name
        return False


    ######################
    ### MAIN FUNCTIONS ###
    ######################

    def run_grasping(self, container = None):
        self.getBestGraspingPoint(container)
        if self.grasp_point is None:
            print('It was suppose to do grasping, but there is no grasp proposal')
            self.execution_possible = False
            return
        if self.visionType != 'real':
            self.callFakeGrasping(prob=0.8, container = container)
            return
        self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                 isExecute=self.isExecute, binId=container, flag=0,
                                 withPause=self.withPause, viz_pub=self.viz_array_pub)
        assert(False)
        self.execution_possible = self.grasping_output['execution_possible']

    def planned_place(self, fixed_container = None):
        fixed_container = [0] #TODO_M: planner only accepts bins 1,2,3 and names them as 0,1,2
        bbox_size = self.bbox_info[7:10] #TODO_M: give bbox_size to the planner!
        
        if self.PlacingPlanner.visionType == 'real': #Update HM
            self.PlacingPlanner.update_real_height_map(fixed_container[0])
        drop_pose = self.PlacingPlanner.place_object_local_best(None, containers = fixed_container) #TODO_M : change placing to return drop_pose
        # Place object using grasping
        placing_output=grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                             isExecute=self.isExecute, binId=fixed_container[0], flag=2, withPause=self.withPause,
                             rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose,
                             viz_pub=self.viz_array_pub, is_drop = False, update_command=update_command)
                             
    def run_stowing(self):
        goToHome.goToARC(slowDown=self.goHomeSlow) # 1. Initialize robot state
        if self.visionType == 'real': # 2. Passive vision update bins
            print("getPassiveVisionEstimate 'update hm sg', '', ", self.tote_ID)
            self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
            number_bins = 2
            for bin_id in range(1,number_bins):
                print("getPassiveVisionEstimate 'update hm', '', ", bin_id)
                self.getPassiveVisionEstimate('update hm', '', bin_id)

        self.num_attempts = 0 
        self.num_attempts_failed = 0
        while True: # 3. Start the stowing loop
            self.num_attempts += 1
            print('-----------------------------\n       RUNNING GRASPING      \n-----------------------------')
            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)
            self.run_grasping(container = self.tote_ID) # 4. Run grasping
            
            if self.execution_possible != False: # 5. Check the weight
                #TODO_M
                ''' 
                self.weight_info[self.tote_ID] = self.weightSensor.readWeightSensor(item_list = [], withSensor=self.withSensorWeight, binNum=self.tote_ID, givenWeights=-11)[self.tote_ID] 
                self.weight_info[self.tote_ID]
                print('Detected weight:',  self.weight_info[self.tote_ID]['weights'])
                
                if self.weight_info[self.tote_ID]['weights'] > 10:
                    self.execution_possible = True
                '''
                if self.execution_possible == None:
                    self.execution_possible = False
                else: # 6. Move to bin 1 for placing
                    grasp(objInput=[], listener=self.listener, br=self.br, isExecute=self.isExecute,
                      binId=1, flag=1, withPause=self.withPause, viz_pub=self.viz_array_pub)
                
            if self.visionType == 'real': # 7. Update vision state of the tote
                self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
            
            print('----------------------------------------------\nExecution_possible = ', self.execution_possible,'\n----------------------------------------------')
            if self.execution_possible: # 8. Placing
                self.fails_in_row = 0
                self.bbox_info = fake_bbox_info_1(self.listener)#Give bounding box to the object 
                self.bbox_info[7:10] = [self.max_dimension, self.max_dimension, self.max_dimension]
                #self.planned_place() #TOdo_M
            else: 
                self.num_attempts_failed += 1                
                self.fails_in_row += 1
                if self.grasp_point is not None: # 9. Add to bad grasp points
                    self.bad_grasping_points.append(self.grasp_point)
                    self.bad_grasping_times.append(time.time())
            if self.fails_in_row > 9: # 10. Failed too many times, take action
                if self.infinite_looping:
                    print('The pick failed 10 times in a row, switching totes')
                    self.switch_tote()
                else:
                    print('The pick failed 10 times in a row, stopping')
                    break
        # Finished stowing
        goToHome.goToARC(slowDown = self.goHomeSlow)
        print("Planner is done")
        
if __name__ == '__main__':
    
    rospy.init_node('Planner')
    
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
    parser.add_option('-e', '--experiment', action='store_true', dest='experiment',
        help='Whether to run passive vision experiments', default=False)
    (opt, args) = parser.parse_args()

    p = TaskPlanner(args)
    p.run_stowing()