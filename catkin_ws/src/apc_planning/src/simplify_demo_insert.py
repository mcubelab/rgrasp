#!/usr/bin/python

import random, time, datetime, json, optparse, rospy, copy, yaml, cv2, math, subprocess
import tf
import ik.visualize_helper
import numpy as np
import os
from ik.marker_helper import createDeleteAllMarker
from grasp_data_recorder import GraspDataRecorder
import signal
import sys
import spatula, gripper
try:
    import passive_vision.srv
except:
    print('FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL')

import sys
sys.path.append(os.environ['CODE_BASE']+'/catkin_ws/src/weight_sensor/src')
sys.path.append(os.environ['HOME'] + '/mcube_learning')
from models.models import two_gelsight_model
import ws_prob
import goToHome
from grasping17 import place, grasp, retrieve, release_safe, grasp_correction
from control_policy import controlPolicy
from ik.helper import fake_bbox_info_1, Timer, vision_transform_precise_placing_with_visualization, get_params_yaml
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError
from gelsight import calibrate_gelsight
import sensor_msgs.msg
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
import pdb
from helper.image_helper import convert_world2image, convert_image2world, translate_image, crop_gelsight, back_substraction, preprocess_image, get_center_of_mass, crop_contact, back_sub_process
import scipy
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

class TaskPlanner(object):
    def __init__(self, opt):
#        self.original_sigint = signal.getsignal(signal.SIGINT)
#        signal.signal(signal.SIGINT, self.stop_running)
        # Class constants
        self.param_grasping = 12
        self.infinite_looping = True
        self.max_dimension = 0.1  #TODO M: adjust for realistic bbox size
        self.goHomeSlow = False
        self.tote_ID = 0
        self.fails_in_row = 0
        self.version = 1.0
        # Configuration
        self.withPause = opt.withPause        
        self.isExecute = opt.isExecute
        self.is_record = opt.is_record        
        self.experiment_type = "is_demo"
        self.experiment_description = "Comments: Demo for object insertion."
        # ROS setup
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #Vision
        self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
        self.all_grasp_proposals = None
        self.grasp_point = None
        self.bridge = CvBridge()
        self.image_topic_list = ["rpi/gelsight/flip_raw_image_basic",  "rpi/gelsight/flip_raw_image2_basic"]
        #Bad points
        self.bad_grasping_points = []
        self.bad_grasping_times = []        
        #Weights
        self.weightSensor = ws_prob.WeightSensor()
        self.withSensorWeight = True
        #Class Publishers
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.proposal_viz_array_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
        self.grasp_status_pub = rospy.Publisher('/grasp_status', sensor_msgs.msg.JointState, queue_size=10, latch=True)
        self.grasp_all_proposals_pub=rospy.Publisher('/grasp_all_proposals',Float32MultiArray,queue_size = 10, latch=True)
        self.grasp_proposal_pub=rospy.Publisher('/grasp_proposal',Float32MultiArray,queue_size = 10, latch=True)
        self.grasp_proposal2_pub=rospy.Publisher('/grasp_proposal2',Float32MultiArray,queue_size = 10, latch=True)
        self.im_input_color_0_pub=rospy.Publisher('/im_input_color_0',Image,queue_size = 10, latch=True)
        self.im_back_color_0_pub=rospy.Publisher('/im_back_color_0',Image,queue_size = 10, latch=True)
        self.im_input_depth_0_pub=rospy.Publisher('/im_input_depth_0',Image,queue_size = 10, latch=True)
        self.im_back_depth_0_pub=rospy.Publisher('/im_back_depth_0',Image,queue_size = 10, latch=True)
        self.im_input_color_1_pub=rospy.Publisher('/im_input_color_1',Image,queue_size = 10, latch=True)
        self.im_back_color_1_pub=rospy.Publisher('/im_back_color_1',Image,queue_size = 10, latch=True)
        self.im_input_depth_1_pub=rospy.Publisher('/im_input_depth_1',Image,queue_size = 10, latch=True)
        self.im_back_depth_1_pub=rospy.Publisher('/im_back_depth_1',Image,queue_size = 10, latch=True)
        self.experiment_comments_pub=rospy.Publisher('/exp_comments',String,queue_size = 10, latch=True)
        self.experiment_type_pub=rospy.Publisher('/exp_type',String,queue_size = 10, latch=True)
        self.objectList_pub=rospy.Publisher('/objectList',String,queue_size = 10, latch=True)
        self.objectType_pub=rospy.Publisher('/objectType',String,queue_size = 10, latch=True)
        rospy.sleep(0.5)

    ###############################
    ### GLOBAL FUNCTIONS ###
    ###############################

    def capture_images(self):
        #capture images
        image_ros_list = []
        image_list = []
        for topic in self.image_topic_list:
            if rospy.get_param('have_robot'):
                image_ros = rospy.wait_for_message(topic, sensor_msgs.msg.Image)
                image_list.append(self.bridge.imgmsg_to_cv2(image_ros, 'rgb8'))
            else:
                image_path = '/media/mcube/data/Dropbox (MIT)/images/gelsight_fingerprint.png'
                image_list.append(cv2.imread(image_path, 1))
        return image_list


    def request_passive_vision_wait(self, bin_id):
        while True:
            try:
                return self.getPassiveVisionEstimate('request', '', bin_id)
            except:
                print('Waiting Vision.')

    def save_passive_vision_images(self, bin_id):
        im_passive_vision = []
        camera_id = 0
        try:
            im_input_color_0_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-input.{}.{}.color.png'.format(bin_id,camera_id), cv2.IMREAD_COLOR), "rgb8")
            im_back_color_0_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-background.{}.{}.color.png'.format(bin_id,camera_id), cv2.IMREAD_COLOR), "rgb8")
            im_input_depth_0_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-input.{}.{}.depth.png'.format(bin_id,camera_id), cv2.IMREAD_GRAYSCALE), "mono8")
            im_back_depth_0_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-background.{}.{}.depth.png'.format(bin_id,camera_id), cv2.IMREAD_GRAYSCALE), "mono8")
            self.im_input_color_0_pub.publish(im_input_color_0_msgs)
            self.im_back_color_0_pub.publish(im_back_color_0_msgs)
            self.im_input_depth_0_pub.publish(im_input_depth_0_msgs)
            self.im_back_depth_0_pub.publish(im_back_depth_0_msgs)

            camera_id = 1
            im_input_color_1_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-input.{}.{}.color.png'.format(bin_id,camera_id), cv2.IMREAD_COLOR), "rgb8")
            im_back_color_1_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-background.{}.{}.color.png'.format(bin_id,camera_id), cv2.IMREAD_COLOR), "rgb8")
            im_input_depth_1_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-input.{}.{}.depth.png'.format(bin_id,camera_id), cv2.IMREAD_GRAYSCALE), "mono8")
            im_back_depth_1_msgs = CvBridge().cv2_to_imgmsg(cv2.imread('/home/mcube/arcdata/tmpdata/'+'passive-vision-background.{}.{}.depth.png'.format(bin_id,camera_id), cv2.IMREAD_GRAYSCALE), "mono8")
            self.im_input_color_1_pub.publish(im_input_color_1_msgs)
            self.im_back_color_1_pub.publish(im_back_color_1_msgs)
            self.im_input_depth_1_pub.publish(im_input_depth_1_msgs)
            self.im_back_depth_1_pub.publish(im_back_depth_1_msgs)
            time.sleep(0.1)
        except:
            print("[Planner] Warning: Your input type is not a numpy array")

    def get_objects(self):
        yaml_content = yaml.load(open(os.environ['CODE_BASE']+"/catkin_ws/src/apc_config/object_properties.yaml"))
        obj_list = yaml_content['/obj'].keys()
        obj_list.sort()
        return obj_list

    def remove_old_points(self, points, times, limit):
        new_points = []
        new_times = []
        for j, point in enumerate(points):
            if time.time() - times[j] < limit: #self.bad_suction_times[j] < time.time() + 60*3: #We only care about things that happened in the last 3 minutes
                new_points.append(point)
                new_times.append(times[j])
        return new_points, new_times

    ########################
    ### VISION FUNCTIONS ###
    ########################

    def call_passive_vision(self, bin_id=0):
        print('Calling passive vision system for grasp proposals. The bin considered is: ', bin_id)
        with Timer('getPassiveVisionEstimate ' + 'request hm sg %d' % bin_id ):
            self.passive_vision_state = self.request_passive_vision_wait(bin_id)

        print('Received grasp proposals.')
        self.all_grasp_proposals = np.asarray(self.passive_vision_state.grasp_proposals)
        #Publish proposals
        grasp_all_proposals_msgs = Float32MultiArray()
        grasp_all_proposals_msgs.data = self.all_grasp_proposals
        self.grasp_all_proposals_pub.publish(grasp_all_proposals_msgs)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/self.param_grasping, self.param_grasping)
        #Sorting all points
        grasp_permutation = self.all_grasp_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]

    def GetGraspPoints(self, num_points, container):
        if self.all_grasp_proposals is None:
            print('I am trying to get proposals')
            self.call_passive_vision(container)
        #Add grasp proposals if possible
        print('There are {} grasp proposals'.format(len(self.all_grasp_proposals)))
        
        ## Filter out outdated bad_grasping_point
        print('There are ', len(self.bad_grasping_points), ' bad_grasping_points ',': ', self.bad_grasping_points)
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
            for i in range(0, len(not_bad_grasp_points_indices)):
                not_bad_grasp_points[i] = self.all_grasp_proposals[not_bad_grasp_points_indices[i], :]
            if len(not_bad_grasp_points) > 0:
                self.all_grasp_proposals = not_bad_grasp_points
            print('bad_points_grasping: ', self.bad_grasping_points)
            num_points_grasp = min(num_points, len(self.all_grasp_proposals))
            self.all_pick_proposals = list(self.all_grasp_proposals[0:num_points_grasp, 0:self.param_grasping])
            self.all_pick_scores = list(self.all_grasp_proposals[0:num_points_grasp, self.param_grasping-1])
        else:
            self.all_pick_proposals = []
            self.all_pick_scores = []

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
            grasp_output = grasp(objInput=grasp_point, listener=self.listener, br=self.br, isExecute=False,
                                   binId=container,  withPause=False, viz_pub=self.viz_array_pub)
            retrieve_output = retrieve(listener=self.listener, br=self.br, isExecute=False,
                                   binId=container,  withPause=False, viz_pub=self.viz_array_pub, ws_object=self.weightSensor)

            if grasp_output['execution_possible'] and retrieve_output['execution_possible']:
                self.grasp_score = copy.deepcopy(self.all_pick_scores[num_it-1])
                self.grasp_point = copy.deepcopy(grasp_point)
                #Visualize grasp
                markers_msg = MarkerArray()
                m0 = createDeleteAllMarker('pick_proposals')
                markers_msg.markers.append(m0)
                for i in range(10):
                    self.proposal_viz_array_pub.publish(markers_msg)
                ik.visualize_helper.visualize_grasping_proposals(self.proposal_viz_array_pub, np.asarray([self.grasp_point]),  self.listener, self.br, True)
                print('Best grasp point:', grasp_point, ' with score: ', self.grasp_score, 'in bin: ', container)
                return
            if grasp_point is not None:
                self.bad_grasping_points.append(grasp_point)
                self.bad_grasping_times.append(-1)
        print('NONE OF THE GRASPING POINTS WORKS')
        self.grasp_point = None
        self.grasp_score = 0
        return

    ######################
    ### MAIN FUNCTIONS ###
    ######################

    def run_grasping(self, container = None):
        self.getBestGraspingPoint(container)
        
        #~Publish grasp proposal information
        grasp_proposal_msgs = Float32MultiArray()
        grasp_proposal_msgs.data = self.grasp_point
        if self.grasp_point is not None:
            self.grasp_proposal_pub.publish(grasp_proposal_msgs)
            comments_msgs = String()
            comments_msgs.data = self.experiment_description
            experiment_type_msgs = String()
            experiment_type_msgs.data = self.experiment_type
            self.experiment_comments_pub.publish(comments_msgs)
            self.experiment_type_pub.publish(experiment_type_msgs)
        else: 
            print('There are no grasp proposal')
            self.execution_possible = False
            return

        #~Visualize grasp proposals
        markers_msg = MarkerArray()
        m0 = createDeleteAllMarker('pick_proposals')
        markers_msg.markers.append(m0)
        for i in range(10):
            self.proposal_viz_array_pub.publish(markers_msg)
        ik.visualize_helper.visualize_grasping_proposals(self.proposal_viz_array_pub, self.all_grasp_proposals,  self.listener, self.br)
        ik.visualize_helper.visualize_grasping_proposals(self.proposal_viz_array_pub, np.asarray([self.grasp_point]),  self.listener, self.br, True)

        #execute for grasp. Stop when the gripper is closed
        self.back_img_list = self.capture_images()

        #self.grasp_point=[0.92737001180648804, -0.391, -0.12441360205411911, 0.0, 0.0, -1.0, 0.067681394517421722, 0.059999998658895493, 0.0, 1.0, 0.0, 0.74888890981674194]

        self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                isExecute=self.isExecute, binId=container, withPause=self.withPause, viz_pub=self.proposal_viz_array_pub,
                recorder=self.gdr)
        num_it = 0
        print('gripper open: ', gripper.getGripperopening())
        is_in_wrong_pose = (gripper.getGripperopening() < 0.025) 
        while is_in_wrong_pose:
            '''
            self.retrieve_output = retrieve(listener=self.listener, br=self.br,
                                       isExecute=self.isExecute, binId=container,
                                       withPause=self.withPause, viz_pub=self.proposal_viz_array_pub,
                                       recorder=self.gdr, ws_object=self.weightSensor, isShake=False)
            self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br,
                                    isExecute=self.isExecute, binId=container, withPause=self.withPause,
                                    viz_pub=self.proposal_viz_array_pub, recorder=self.gdr, open_hand=False)
            '''
            # Motion heuristic
            initial_dz = 0.1
            dz = .015  #should be related to object length
            ik.helper.move_cart(dz=initial_dz); rospy.sleep(0.5)
            ik.helper.move_cart(dz=-initial_dz);rospy.sleep(0.5)
            gripper.open()
            ik.helper.move_cart(dz=dz); rospy.sleep(0.5)
            #should be done in the direction of the gripper plane
            dx = .02*num_it
            ik.helper.move_cart_hand(self.listener, dx=dx, dy=0, dz=0);rospy.sleep(0.5)
            ik.helper.move_cart_hand(self.listener, dx=-2*dx, dy=0, dz=0);rospy.sleep(0.5)
            ik.helper.move_cart_hand(self.listener, dx=dx, dy=0, dz=0);rospy.sleep(0.5)
            ik.helper.move_cart(dz=-dz);
            gripper.close()
            self.gdr.save_data_recorded = False
            num_it +=1
            is_in_wrong_pose = (gripper.getGripperopening() < 0.025) #and (gripper.getGripperopening() > 0.015) 
        
        self.retrieve_output = retrieve(listener=self.listener, br=self.br,
                                 isExecute=self.isExecute, binId=container,
                                 withPause=self.withPause, viz_pub=self.proposal_viz_array_pub,
                                 recorder=self.gdr, ws_object=self.weightSensor, isShake=False)
        self.execution_possible = self.retrieve_output['execution_possible']

    def planned_place(self):
        # Place approx. at the center bin
        binId = 0
        drop_pose = get_params_yaml('bin{}_pose'.format(binId))
        drop_pose[1] = drop_pose[1] + 0.03

        # Place depending on the position of the COM of the contact patch
        image_list = self.capture_images()
        img0, aux, contact = crop_contact(self.back_img_list[0], image_list[0], gel_id = 1, is_zeros=True)
        img1, aux, contact = crop_contact(self.back_img_list[1], image_list[1], gel_id = 2, is_zeros=True)
        
        img0 = scipy.misc.imresize(img0, (224,224,3))
        img1 = scipy.misc.imresize(img1, (224,224,3))
        
        pos_pixel_img0 = get_center_of_mass(img0)
        pos_pixel_img1 = get_center_of_mass(img1)
        pos_pixel_avg = np.mean( np.array([pos_pixel_img0, pos_pixel_img1 ]), axis=0)
        # Compute actual motion for the robot and check if there is collision (x in pixel frame -> y in hand frame     y in pixel frame -> -z in hand frame)
        pos_world_avg = np.nan_to_num(convert_image2world([225/2-pos_pixel_avg[1], 0])) #np.array([y,-z])
        print('Pixel positions: ', pos_pixel_avg, '. World positions: ', pos_world_avg)
        drop_pose[1] = drop_pose[1] + pos_world_avg[0]
        print(drop_pose)
        # Place object using graspingc
        self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info,viz_pub=self.viz_array_pub,listener=self.listener)
        place(listener=self.listener, br=self.br,
                         isExecute=self.isExecute, binId=binId,  withPause=self.withPause,
                         rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose,
                         viz_pub=self.viz_array_pub, is_drop = False, recorder=self.gdr)

    def run_experiments(self):
        #######################
        ## initialize system ##
        #######################
        #Get object list
        obj_list = self.get_objects()
        print(obj_list)
        obj_ans = raw_input('Are these the objects?(y/n)')
        while obj_ans != 'y':
            obj_list = self.get_objects()
            print(obj_list)
            obj_ans = raw_input('Are these the objects?(y/n)')

        objectList_msgs = String()
        objectList_msgs.data = '\n'.join(obj_list)
        self.objectList_pub.publish(objectList_msgs) #HACK
        
        # Start execution
        goToHome.goToARC(slowDown=self.goHomeSlow) # 1. Initialize robot state
        print("getPassiveVisionEstimate 'update hm sg', '', ", self.tote_ID)
        self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
        self.save_passive_vision_images(self.tote_ID)
        
        ##################
        ## Manipulation loop ##
        ##################
        if (rospy.get_param('have_robot')==True and self.is_record==True):
            directory='/media/mcube/data/Dropbox (MIT)/rgrasp_dataset'
            assert(directory)
            self.gdr = GraspDataRecorder(directory=directory) #We instantiate the recorder
        else:
            self.gdr = None
        while True:
            print('-----------------------------\n       RUNNING GRASPING      \n-----------------------------')
            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)
            self.all_grasp_proposals = None
            self.run_grasping(container = self.tote_ID) # 4. Run grasping

            self.obj_ID = 'no_item'
            self.obj_weight = 0
            print('-----------------------------\n Execution_possible according to primitive = {} \n-----------------------------'.format(self.execution_possible) )
            if self.execution_possible != False: # 5. Check the weight
                self.weight_info = self.weightSensor.readWeightSensor(item_list = obj_list, 
                                                    withSensor=self.withSensorWeight, binNum=self.tote_ID, givenWeights=-11)
                self.obj_weight = self.weight_info['weights']
                print('Detected weight:',  self.obj_weight)
                if self.obj_weight > 10: # Weight need to be higher than 10g
                    self.execution_possible = True
                #Identify object
                max_prob_index = (self.weight_info['probs']).tolist().index(max(self.weight_info['probs']))
                self.obj_ID = obj_list[max_prob_index]
                if self.obj_ID == 'no_item' or self.execution_possible == None:
                    self.execution_possible = False
                print('-----------------------------\n Execution_possible according to weight = {} \n-----------------------------'.format(self.execution_possible) )        
                #Manually change execution result
                self.execution_possible = raw_input('Did it succeeded?')

            #Publish experiment outcome
            grasp_status_msgs = sensor_msgs.msg.JointState()
            grasp_status_msgs.name = ['grasp_success', 'code_version', 'tote_ID', 'detected_weight'] #grasp proposals, grasp_point, scores, score,
            grasp_status_msgs.position = [self.execution_possible, self.version, self.tote_ID, self.obj_weight]

            object_ID_msgs = String()
            object_ID_msgs.data = self.obj_ID 

            if self.grasp_point is not None and rospy.get_param('have_robot'):
                self.grasp_status_pub.publish(grasp_status_msgs)
                self.objectType_pub.publish(object_ID_msgs)
                self.gdr.update_topic(key='grasp_status')
                self.gdr.update_topic(key='objectType')

            print('-----------------------------\n Execution_possible = {} \n-----------------------------'.format(self.execution_possible) )
            if self.execution_possible: # 8. Placing
                self.fails_in_row = 0
                self.bbox_info = fake_bbox_info_1(self.listener)#Give bounding box to the object
                self.bbox_info[7:10] = [self.max_dimension, self.max_dimension, self.max_dimension]
                self.planned_place() #TODO_M
            else:
                self.fails_in_row += 1
                gripper.open()
                spatula.open()
                if self.grasp_point is not None: # 9. Add to bad grasp points
                    self.bad_grasping_points.append(self.grasp_point)
                    self.bad_grasping_times.append(time.time())

            print('************************\n         End of grasping attempt         \n***********************************')
            # 7. Update vision state of the tote
            self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
            self.save_passive_vision_images(self.tote_ID)

            if self.fails_in_row > 4: # 10. Failed too many times, take action
                print('The pick failed 4 times in a row, stopping if not infinite looping')
                if not self.infinite_looping:
                    break
        # Finished stowing
        goToHome.goToARC(slowDown=self.goHomeSlow)
        print("Planner is done")

    def stop_running(self, signum, frame):
        signal.signal(signal.SIGINT, self.original_sigint)

        print('Program manually stopped')

        self.gdr.kill_recorder()

        signal.signal(signal.SIGINT, self.stop_running)

if __name__ == '__main__':
    # Kill python programs nvidia
    import re, subprocess
    out_process = subprocess.check_output(["echo  $(nvidia-smi | grep python )"], shell=True)
    list_int = map(int, re.findall(r'\d+', out_process))
    kill_process = [i for i in list_int if i >= 1000]
    for i in kill_process:
        os.system("kill -9 {}".format(i))
    
    rospy.init_node('Planner')
    
    parser = optparse.OptionParser()
    parser.add_option('-p', '--pause', action='store_true', dest='withPause',
        help='To pause or not', default=False)
    parser.add_option('-n', '--noexe', action='store_false', dest='isExecute',
        help='To execute or not', default=True)
    parser.add_option('-b', '--record_data', action='store', dest='is_record',
        help='Turn data recording on/off?', default=True)
    (opt, args) = parser.parse_args()
    p = TaskPlanner(opt)
    p.run_experiments()
