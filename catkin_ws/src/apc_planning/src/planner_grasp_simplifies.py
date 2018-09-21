#!/usr/bin/python


from placing_grasp import PlacingPlanner
import random, time, datetime, json, optparse, rospy, copy, yaml, cv2, math
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
sys.path.append('/home/mcube/silhouettes/gathering')
from control_robot import ControlRobot
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
#from control_policy import capture_images
from helper.image_helper import convert_world2image, convert_image2world, translate_image, crop_gelsight, back_substraction, preprocess_image, get_center_of_mass, crop_contact, back_sub_process
import scipy


import subprocess, os, signal
import numpy as np
def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)
            
def start_ros_bag(bagfilename, topics, dir_save_bagfile):
    subprocess.Popen('rosbag record -q -O %s %s' % (bagfilename, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)



class TaskPlanner(object):
    def __init__(self, opt):
        self.original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self.stop_running)

        self.passiveVisionTypes = {'real' : self.call_passive_vision,
                                   'file' : self.call_passive_vision,
                                   'virtual' : self.callFakePassiveVision}
        # Class constants
        self.param_grasping = 12
        self.infinite_looping = True
        self.max_dimension = 0.1  #TODO M: adjust for realistic bbox size
        self.goHomeSlow = False
        self.tote_ID = 0
        self.fails_in_row = 0
        self.switch_dict = {0:1,1:0}
        self.version = 1.0
        self.experiment_description = "Comments: Data collection for water bottle with no policy."
        # Configuration
        self.withPause = opt.withPause
        self.experiment = opt.experiment
        self.isExecute = opt.isExecute
        self.add_noise = opt.add_noise
        self.is_hand = opt.is_hand
        self.is_record = opt.is_record
        self.is_control = opt.is_control
        self.smirror = True
        self.use_COM = False
        self.use_raw = True
        self.grasp_std = [0,0,0,0]
        self.grasp_noise = [0,0,0, 0]
        if self.is_control:
            self.experiment_type = "is_control"
        else:
            self.experiment_type = "is_data_collection"
        # ROS setup
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        #Vision
        self.haverobot = rospy.get_param('/have_robot', True)
        self.visionType = opt.visionType
        if not self.haverobot:
            self.visionType = 'virtual'
        if self.visionType == 'real':
            self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
        self.FAKE_PASSIVE_VISION_DIR = os.environ['CODE_BASE'] + '/input/fake_dirs/fake_passive_vision/'
        self.FAKE_GRASPING_DIR = os.environ['CODE_BASE'] + '/input/fake_dirs/fake_grasping/'
        self.passive_vision_file_id = opt.passive_vision_file_id
        self.all_grasp_proposals = None
        self.grasp_noise = [0,0,0,0]
        self.grasp_point = None
        #Bad points
        self.bad_grasping_points = []
        self.bad_grasping_times = []
        #Placing
        self.PlacingPlanner = PlacingPlanner(visionType = self.visionType)
        #Control policy
        if self.is_control:
            self.model = two_gelsight_model()
            self.controller = controlPolicy(self.model, ["rpi/gelsight/flip_raw_image",  "rpi/gelsight/flip_raw_image2"], self.listener, self.br)
        #Weights
        self.weightSensor = ws_prob.WeightSensor()
        self.withSensorWeight = True
        if self.visionType != 'real':
            self.withSensorWeight = False
        self.weight_info = [None for _ in range(9)]  #Adding the boxes
        #Class Publishers
        self.viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.proposal_viz_array_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
        self.grasp_status_pub = rospy.Publisher('/grasp_status', sensor_msgs.msg.JointState, queue_size=10, latch=True)
        self.grasp_all_proposals_pub=rospy.Publisher('/grasp_all_proposals',Float32MultiArray,queue_size = 10, latch=True)
        self.grasp_proposal_pub=rospy.Publisher('/grasp_proposal',Float32MultiArray,queue_size = 10, latch=True)
        self.grasp_noise_pub=rospy.Publisher('/grasp_noise',Float32MultiArray,queue_size = 10, latch=True)
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
        # self.grasp_noise_pub=rospy.Publisher('/grasp_noise',Float32MultiArray,queue_size = 10, latch=True)
        # self.grasp_noise_std_dev_pub=rospy.Publisher('/grasp_noise_std_dev',Float32MultiArray,queue_size = 10, latch=True)
        self.objectList_pub=rospy.Publisher('/objectList',String,queue_size = 10, latch=True)
        self.objectType_pub=rospy.Publisher('/objectType',String,queue_size = 10, latch=True)
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

    def switch_tote(self):
        self.tote_ID = 1-self.tote_ID
        #~frank edit: continuous switch between totes 0 and 1
#        self.tote_ID = self.switch_dict[self.tote_ID]
        self.fails_in_row = 0
        if self.visionType == 'real': # 2. Passive vision update bins
            number_bins = 2
            for bin_id in range(number_bins):
                print("getPassiveVisionEstimate 'update hm sg', '', ", bin_id)
                self.getPassiveVisionEstimate('update hm sg', '', bin_id)


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
        #Publish proposals
        grasp_all_proposals_msgs = Float32MultiArray()
        grasp_all_proposals_msgs.data = self.all_grasp_proposals
        if self.all_grasp_proposals is not None:
            self.grasp_all_proposals_pub.publish(grasp_all_proposals_msgs)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/self.param_grasping, self.param_grasping)

    def callFakeGrasping(self, prob, container):
        print('------- DOING GRASPING ------- ')
        print(' grasp_point = ', self.grasp_point)
        grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute,
              binId=container,  withPause=False, viz_pub=self.viz_array_pub)

        if self.is_control:
            #find new and improved grasp points
            back_img_list = self.controller.capture_images()
            best_grasp_dict = self.controller.control_policy(back_img_list, smirror=self.smirror, use_COM=self.use_COM)
            # self.controller.visualize_actions(with_CAM = False)
            # self.controller.visualize_best_action()
            print best_grasp_dict['delta_pos']

            #go for new grasp Point
            self.grasping_output = grasp_correction(self.grasp_point, best_grasp_dict['delta_pos'], self.listener, self.br)

        retrieve(listener=self.listener, br=self.br, isExecute=self.isExecute,
              binId=container,  withPause=False, viz_pub=self.viz_array_pub, ws_object=self.weightSensor)

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
        print self.passive_vision_state
        print('Received grasp proposals.')
        self.all_grasp_proposals = np.asarray(self.passive_vision_state.grasp_proposals)
        #Publish proposals
        grasp_all_proposals_msgs = Float32MultiArray()
        grasp_all_proposals_msgs.data = self.all_grasp_proposals
        self.grasp_all_proposals_pub.publish(grasp_all_proposals_msgs)
        self.all_grasp_proposals = self.all_grasp_proposals.reshape(len(self.all_grasp_proposals)/self.param_grasping, self.param_grasping)
#        visualize_grasping_proposals(self.proposal_viz_array_pub, self.all_grasp_proposals, False)
        #Sorting all points
        grasp_permutation = self.all_grasp_proposals[:,self.param_grasping-1].argsort()[::-1]
        self.all_grasp_proposals = self.all_grasp_proposals[grasp_permutation]

    def GetGraspPoints(self, num_points, container):
        if self.all_grasp_proposals is None:
            print('I am trying to get proposals')
            self.passiveVisionTypes[self.visionType](container)
        #Add grasp proposals if possible
        print('There are {} grasp proposals'.format(len(self.all_grasp_proposals)))
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

    def noise_initialize(self):
        max_length = max(rospy.get_param('obj/{}/dimensions'.format(self.get_objects()[0]))) #TODO: only valid for 1 object
        min_length = min(rospy.get_param('obj/{}/dimensions'.format(self.get_objects()[0]))) #TODO: only valid for 1 object
        gel_length = 0.05
        # interval = max_length/2. + gel_length/2. - min_length/2.
        interval = gel_length
        std_x = 0.0
        # std_y = 0.07 #
        std_ori = 0.
        std_width = 0.0
        noise_x = np.random.uniform(-std_x,std_x)
        # noise_y =  (np.random.randint(2)*2-1)*(min_length/2. - gel_length/4.+ np.random.uniform(0,interval)) #np.random.uniform(-std_y,std_y)
        noise_y =  1*(np.random.randint(2)*2-1)*(max_length/2. - gel_length/2.+ np.random.uniform(0,interval)) #np.random.uniform(-std_y,std_y)
        noise_ori = np.random.uniform(-std_ori,std_ori)
        noise_width = np.random.uniform(-std_width,std_width)
        self.grasp_std = [std_x,gel_length/2., std_ori, std_width]
        self.grasp_noise = [0,0,0, 0]#[noise_x,noise_y, noise_ori, noise_width]
        # self.grasp_noise_std_dev_pub.publish(np.asarray(self.grasp_std))
        # self.grasp_noise_pub.publish(np.asarray(self.grasp_noise))
        return

    def perturb_grasp_point(self):
        ### Add some random noise
        #grasp properties: [surfaceCentroid,graspDirection,graspDepth,graspJawWidth,gripperAngleDirection,graspConf]];
        #grasp properties: x,y,z,[0,0,-1],depth, width_gripper,angledirection, score
        ## Update
        self.noise_initialize()
        #x
        self.grasp_point[0] = self.grasp_point[0]+self.grasp_noise[0]
        #y
        self.grasp_point[1] = self.grasp_point[1]+self.grasp_noise[1]
        #width
        self.grasp_point[7] = self.grasp_point[7]+self.grasp_noise[3]
        #ori
        initial_ori = math.acos(self.grasp_point[9])*180/math.pi
        new_ori = initial_ori + self.grasp_noise[2]
        rad_new_ori = new_ori*math.pi/180
        self.grasp_point[8] = -math.sin(rad_new_ori)
        self.grasp_point[9] = math.cos(rad_new_ori)
        ## if needed, we could compute the new score of the proposal...
        return

    def perturb_grasp_point_hand_frame(self):
        ### Add some random noise in the hand frame as defined in RVIZ link_6
        self.noise_initialize()
        #get grasp pose
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = ik.helper.get_picking_params_from_12(self.grasp_point)
        hand_orient_norm = np.vstack([hand_X,hand_Y,hand_Z])
        hand_orient_norm=hand_orient_norm.transpose()
        hand_orient_quat=ik.helper.mat2quat(hand_orient_norm)
        grasp_pose = np.zeros(7)
        grasp_pose[0:3] = self.grasp_point[0:3]
        grasp_pose[3:7] = hand_orient_quat
        #publish new frame
        ik.roshelper.pubFrame(self.br, pose=grasp_pose, frame_id='hand_frame', parent_frame_id='map', npub=5)
        #perturn grasp
        delta_pose_hand = np.zeros((7))
        delta_pose_hand[0:3] = np.array([self.grasp_noise[0], self.grasp_noise[1], 0])
        delta_pose_hand[3:7] = np.array([0,0,0,1])
        #convert perturbation back to world frame
        pose_world = ik.roshelper.poseTransform(delta_pose_hand, "hand_frame", "map", self.listener)
        #overwrite original grasp proposal
        self.grasp_point[0] = pose_world[0]
        self.grasp_point[1] = pose_world[1]
        self.grasp_point[7] = self.grasp_point[7]+self.grasp_noise[3]
        #ori
        #TODO
        return
        

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
                self.grasp_point = copy.deepcopy(grasp_point) #
                #self.grasp_point = [0.92737001180648804, -0.30099999904632568, -0.14360031485557556, 0.0, 0.0, -1.0, 0.048494685441255569, 0.079999998211860657, 0.0, 1.0, 0.0, 0.74666666984558105]
                self.grasp_point = [0.92737001180648804, -0.42100000381469727, -0.091183662414550781, 0.0, 0.0, -1.0, 0.10091133415699005, 0.059999998658895493, -0.70710676908493042, -0.70710676908493042, 0.0, 0.65777778625488281]
                #self.grasp_point = [1.0073699951171875, -0.40099999308586121, -0.088611423969268799, 0.0, 0.0, -1.0, 0.10348358005285263, 0.059999998658895493, 0.0, 1.0, 0.0, 0.70444446802139282]
                #Visualize before adding noise
                markers_msg = MarkerArray()
                m0 = createDeleteAllMarker('pick_proposals')
                markers_msg.markers.append(m0)
                for i in range(10):
                    self.proposal_viz_array_pub.publish(markers_msg)
                ik.visualize_helper.visualize_grasping_proposals(self.proposal_viz_array_pub, np.asarray([self.grasp_point]),  self.listener, self.br, True)
                print ('[Planner]*************************************** Add noise*****************')
                print ('[Planner]', self.add_noise)
                if self.add_noise:
                    if self.is_hand:
                        self.perturb_grasp_point_hand_frame()
                    else:
                        self.perturb_grasp_point()
                
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
        grasp_proposal_msgs = Float32MultiArray()
        grasp_proposal_msgs.data = self.grasp_point
        grasp_noise_msgs = Float32MultiArray()
        grasp_noise_msgs.data = self.grasp_noise

        if self.grasp_point is not None:
            self.grasp_proposal_pub.publish(grasp_proposal_msgs)
            self.grasp_noise_pub.publish(grasp_noise_msgs)
        comments_msgs = String()
        comments_msgs.data = self.experiment_description
        experiment_type_msgs = String()
        experiment_type_msgs.data = self.experiment_type
        self.experiment_comments_pub.publish(comments_msgs)
        self.experiment_type_pub.publish(experiment_type_msgs)

        if self.grasp_point is None:
            print('It was suppose to do grasping, but there is no grasp proposal')
            self.execution_possible = False
            return
        if self.visionType != 'real':
            self.callFakeGrasping(prob=0.8, container = container)
            return

        #~visualize grasp proposals
        markers_msg = MarkerArray()
        m0 = createDeleteAllMarker('pick_proposals')
        markers_msg.markers.append(m0)
        for i in range(10):
            self.proposal_viz_array_pub.publish(markers_msg)
        ik.visualize_helper.visualize_grasping_proposals(self.proposal_viz_array_pub, self.all_grasp_proposals,  self.listener, self.br)
        ik.visualize_helper.visualize_grasping_proposals(self.proposal_viz_array_pub, np.asarray([self.grasp_point]),  self.listener, self.br, True)

        #execute for grasp. Stop when the gripper is closed
        if self.is_control:
            back_img_list = self.controller.capture_images()

        #self.grasp_point=[0.92737001180648804, -0.391, -0.12441360205411911, 0.0, 0.0, -1.0, 0.067681394517421722, 0.059999998658895493, 0.0, 1.0, 0.0, 0.74888890981674194]
#        self.background_images = self.capture_images() #TODO: commented to make it work
        import pdb; pdb.set_trace()
        #Record bag
        topics = ['/robot1_RRICartState', '/robot1_RRIJointState', '/rpi/gelsight/flip_raw_image', '/tf', '/arc_1/rgb_bin0']
        dir_save_bagfile = '/media/mcube/data/shapes_data/graps/'
        bagfilename = 'first_attempt_{}.bag'.format(time.time())
        rosbag_proc = start_ros_bag(bagfilename, topics, dir_save_bagfile) 
        rospy.sleep(0.5) 
        
        self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute, binId=container, withPause=self.withPause, viz_pub=self.proposal_viz_array_pub, recorder=self.gdr)

        if self.is_record==True:
            self.gdr.save_item(item_name='grasp_noise_std_dev', data=self.grasp_std)
            self.gdr.save_item(item_name='grasp_noise', data=self.grasp_noise)
        is_in_wrong_pose = (gripper.getGripperopening() < 0.005) #raw_input()
        if is_in_wrong_pose:
            self.retrieve_output = retrieve(listener=self.listener, br=self.br,
                                     isExecute=self.isExecute, binId=container,
                                     withPause=self.withPause, viz_pub=self.proposal_viz_array_pub, recorder=self.gdr, ws_object=self.weightSensor)
            #gripper.open()
            gripper.close()                         
            self.grasping_output = grasp(objInput=self.grasp_point, listener=self.listener, br=self.br, isExecute=self.isExecute, binId=container, withPause=self.withPause, viz_pub=self.proposal_viz_array_pub, recorder=self.gdr)
            
        if self.is_control:
            if gripper.getGripperopening() > 0.017:
                print ('[Planner]: ', gripper.getGripperopening())
                # WE PAUSE THE RECOORDER TO SAVE DATA
                #self.gdr.pause_recording()
                #find new and improved grasp points
                best_grasp_dict, initial_score = self.controller.control_policy(back_img_list, smirror=self.smirror, use_COM = self.use_COM, use_raw = self.use_raw)


                # self.controller.visualize_actions(with_CAM = False)
                #self.controller.visualize_best_action(with_CAM = False)
                #pdb.set_trace()
                #save network information action_dict and best_action_dict
                #WE UNPAUSE THE RECORDER
                #self.gdr.replay_recording()
                self.gdr.save_item(item_name='initial_score', data=initial_score)
                self.gdr.save_item(item_name='best_grasp_dict', data=best_grasp_dict)
                self.gdr.save_item(item_name='action_dict', data=self.controller.action_dict)
                self.gdr.save_item(item_name='best_action_dict', data=self.controller.best_action_dict)
                #go for new grasp PointgraspPose
                self.grasping_output = grasp_correction(self.grasp_point, best_grasp_dict['delta_pos'], self.listener, self.br)
                second_best_grasp_dict, final_score = self.controller.control_policy(back_img_list, smirror=self.smirror, use_COM = self.use_COM, use_raw = self.use_raw)
                self.gdr.save_item(item_name='final_score', data=final_score)
                self.gdr.save_item(item_name='second_best_grasp_dict', data=second_best_grasp_dict)
                self.gdr.save_data_recorded = True


            else:
                self.gdr.save_data_recorded = False
        #frank hack for double grasping
        if self.experiment_type == 'is_data_collection':
            if gripper.getGripperopening() > 0.017:
                # self.grasping_output = grasp_correction(self.grasp_point, np.array([0,0,0]), self.listener, self.br)
                self.gdr.save_data_recorded = True
            else:
                self.gdr.save_data_recorded = False


        self.retrieve_output = retrieve(listener=self.listener, br=self.br,
                                 isExecute=self.isExecute, binId=container,
                                 withPause=self.withPause, viz_pub=self.proposal_viz_array_pub, recorder=self.gdr, ws_object=self.weightSensor)
        self.execution_possible = self.retrieve_output['execution_possible']

    def planned_place(self):
        if len(self.get_objects()) == 1:
            fixed_container = [self.tote_ID]
        else:
            fixed_container = [1-self.tote_ID] #TODO_M: planner only accepts bins 1,2,3 and names them as 0,1,2

        if self.PlacingPlanner.visionType == 'real': #Update HM
            self.PlacingPlanner.update_real_height_map(fixed_container[0])

        #Get obj dimensions:
        try:
            asking_for = '/obj/'+self.obj_ID
            aux_obj = rospy.get_param(asking_for)
            obj_dim = aux_obj['dimensions']
        except:
            print('Could not get the object dimensions')
            obj_dim=[0.12,0.12,0.12]
        drop_pose = self.PlacingPlanner.place_object_local_best(obj_dim=obj_dim, containers = fixed_container) #TODO_M : change placing to return drop_pose
        print('drop_pose', drop_pose)

        #~frank hack: drop pose
        drop_pose = get_params_yaml('bin'+str(fixed_container[0])+'_pose')
        drop_pose[1] = drop_pose[1] + 0.03

        ## TODO add here a correction depending on the com of the contact patch of the object?
        image_list = self.controller.capture_images()
        image_list[0] = crop_contact(self.background_images[0], image_list[0], gel_id = 1, is_zeros=use_COM)
        image_list[1] = crop_contact(self.background_images[1], image_list[1], gel_id = 2, is_zeros=use_COM)
        img0=scipy.misc.imresize(image_list[0], (224,224,3))
        img1=scipy.misc.imresize(image_list[1], (224,224,3))
        
        pos_pixel_img0 = get_center_of_mass(img0)
        pos_pixel_img1 = get_center_of_mass(img1)
        pos_pixel_avg = np.mean( np.array([pos_pixel_img0, pos_pixel_img1 ]), axis=0 )  #TODO: check axis


        #x in pixel frame -> y in hand frame
        #y in pixel frame -> -z in hand frame
        # Compute actual motion for the robot and check if there is collision
        
        pos_world_avg = convert_image2world([225/2-pos_pixel_avg[1], 0]) #np.array([y,-z])
        delta_pos_avg = np.array([0,-pos_world_avg[0],0])
        print('Pixel positions: ', pos_pixel_avg, '. World positions: ', pos_world_avg)
        drop_pose[1] = drop_pose[1] + pos_world_avg[0]
        
        # Place object using graspingc
        self.rel_pose, self.BoxBody=vision_transform_precise_placing_with_visualization(self.bbox_info,viz_pub=self.viz_array_pub,listener=self.listener)
        place(listener=self.listener, br=self.br,
                         isExecute=self.isExecute, binId=fixed_container[0],  withPause=self.withPause,
                         rel_pose=self.rel_pose, BoxBody=self.BoxBody, place_pose=drop_pose,
                         viz_pub=self.viz_array_pub, is_drop = False, recorder=self.gdr)

    def run_experiments(self):
        #######################
        ## initialize system ##
        #######################
        #Get object list
        obj_list = self.get_objects()
        print(obj_list)
        #assert(False)
        obj_ans = raw_input('Are these the objects?(y/n)')
        while obj_ans != 'y':
            obj_list = self.get_objects()
            print(obj_list)
            obj_ans = raw_input('Are these the objects?(y/n)')

        objectList_msgs = String()
        objectList_msgs.data = '\n'.join(obj_list)
        #HACK
        self.objectList_pub.publish(objectList_msgs)
        goToHome.goToARC(slowDown=self.goHomeSlow) # 1. Initialize robot state
        # calibrate_gelsight()
        num_grasps=0
        if self.visionType == 'real': # 2. Passive vision update bins
            number_bins = 2
            im_passive_vision = []
            for bin_id in range(number_bins):
                print("getPassiveVisionEstimate 'update hm sg', '', ", bin_id)
                self.getPassiveVisionEstimate('update hm sg', '', bin_id)
            self.save_passive_vision_images(self.tote_ID)
        ##################
        ## stowing loop ##
        ##################
        if (rospy.get_param('have_robot')==True and self.is_record==True):
            directory='/media/mcube/data/Dropbox (MIT)/rgrasp_dataset'
            assert(directory)
            self.gdr = GraspDataRecorder(directory=directory) #We instantiate the recorder
        else:
            self.gdr = None
        self.num_attempts = 0
        self.num_attempts_failed = 0
        while True:
            if num_grasps==100:
                calibrate_gelsight()
                num_grasps=0
            self.num_attempts += 1
            print('-----------------------------\n       RUNNING GRASPING      \n-----------------------------')
            self.weightSensor.calibrateWeights(withSensor=self.withSensorWeight)
            self.all_grasp_proposals = None
            
            
            
            
            self.run_grasping(container = self.tote_ID) # 4. Run grasping

            self.obj_ID = 'no_item'
            self.obj_weight = 0
            cr = ControlRobot(gs_ids=[1], force_list=[25], listener=True)
            path = '/media/mcube/data/shapes_data/graps/grasp_data_{}/'.format(time.time())
            cr.palpate(speed=200, force_list=cr.force_list, save=True, path=path, save_only_picture=False, i=0)
            import pdb; pdb.set_trace()
            if 0 and self.execution_possible != False: # 5. Check the weight
                self.weight_info[self.tote_ID] = self.weightSensor.readWeightSensor(item_list = obj_list, withSensor=self.withSensorWeight, binNum=self.tote_ID, givenWeights=-11)
                print('-----------------------------\n Execution_possible according to primitive = {} \n-----------------------------'.format(self.execution_possible) )
                print('Detected weight:',  self.weight_info[self.tote_ID]['weights'])
                self.obj_weight = self.weight_info[self.tote_ID]['weights']
                if self.obj_weight > 10:
                    self.execution_possible = True

                #Identify object
                max_prob_index = (self.weight_info[self.tote_ID]['probs']).tolist().index(max(self.weight_info[self.tote_ID]['probs']))
                self.obj_ID = obj_list[max_prob_index]
                if self.obj_ID == 'no_item':
                    self.execution_possible = False
                if self.execution_possible == None:
                    self.execution_possible = False
#            if self.visionType == 'real': # 7. Update vision state of the tote
#                self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
#                self.save_passive_vision_images(self.tote_ID)
            # end bag recording
            terminate_ros_node("/record")   
            rospy.sleep(10)
            #Publish experiment outcome
            grasp_status_msgs = sensor_msgs.msg.JointState()
            grasp_status_msgs.name = ['grasp_success', 'code_version', 'tote_ID', 'detected_weight'] #grasp proposals, grasp_point, scores, score,
            grasp_status_msgs.position = [self.execution_possible, self.version, self.tote_ID, self.obj_weight]

            object_ID_msgs = String()
            object_ID_msgs.data = self.obj_ID #, self.obj_weight]

            if self.grasp_point is not None and rospy.get_param('have_robot'):
                self.grasp_status_pub.publish(grasp_status_msgs)
                self.objectType_pub.publish(object_ID_msgs)
                self.gdr.update_topic(key='grasp_status')
                self.gdr.update_topic(key='objectType')

            print('-----------------------------\n Execution_possible = {} \n-----------------------------'.format(self.execution_possible) )
            if self.execution_possible: # 8. Placing
                num_grasps+=1
                self.fails_in_row = 0
                self.bbox_info = fake_bbox_info_1(self.listener)#Give bounding box to the object
                self.bbox_info[7:10] = [self.max_dimension, self.max_dimension, self.max_dimension]
                self.planned_place() #TODO_M
            else:
                self.num_attempts_failed += 1
                self.fails_in_row += 1
                #gripper.open()
                spatula.open()
                if self.grasp_point is not None: # 9. Add to bad grasp points
                    self.bad_grasping_points.append(self.grasp_point)
                    self.bad_grasping_times.append(time.time())

            print('***********************************************************')
            if self.visionType == 'real': # 7. Update vision state of the tote
                self.getPassiveVisionEstimate('update hm sg', '', self.tote_ID)
                self.save_passive_vision_images(self.tote_ID)

            if self.fails_in_row > 3: # 10. Failed too many times, take action
                if self.infinite_looping and len(obj_list) > 1:
                    #self.switch_tote()
                    print('The pick failed 4 times in a row, switching totes, the tote id is {}'.format(self.tote_ID))
                else:
                    print('The pick failed 4 times in a row, stopping')
                    break
        # Finished stowing
        goToHome.goToARC(slowDown = self.goHomeSlow)
        print("Planner is done")

    def stop_running(self, signum, frame):
        signal.signal(signal.SIGINT, self.original_sigint)

        print('Program manually stopped')

        self.gdr.kill_recorder()

        signal.signal(signal.SIGINT, self.stop_running)

if __name__ == '__main__':
    
    
    print 'start record'
    
    topics = ['/robot1_RRICartState', '/robot1_RRIJointState', '/rpi/gelsight/flip_raw_image', '/tf', '/arc_1/rgb_bin0']
    dir_save_bagfile = '/media/mcube/data/shapes_data/graps/'
    bagfilename = 'gs_showoff.bag'
    rosbag_proc = start_ros_bag(bagfilename, topics, dir_save_bagfile) 
    rospy.sleep(0.5) 
    
    raw_input('Done?')
    
        # end bag recording
    terminate_ros_node("/record") 
    
    rospy.sleep(50)
    
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
    parser.add_option('-i', '--item', action='store', dest='objectType',
        help='Name object considered', default='None')
    parser.add_option('-r', '--random_noise', action='store', dest='add_noise',
        help='Add random noise to grasp proposal?', default=False)
    parser.add_option('-f', '--hand_frame', action='store', dest='is_hand',
        help='Hand frame vs. world frame', default=False)
    parser.add_option('-b', '--record_data', action='store', dest='is_record',
        help='Turn data recording on/off?', default=True)
    parser.add_option('-c', '--control', action='store', dest='is_control',
        help='Turn control policy on/off?', default=False)
    (opt, args) = parser.parse_args()

    p = TaskPlanner(opt)

    p.run_experiments()
