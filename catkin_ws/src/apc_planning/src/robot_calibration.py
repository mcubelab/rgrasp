#!/usr/bin/env python

from numpy.linalg import inv
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from scipy import interpolate

import yaml
import rospy
import numpy as np
from numpy import linalg as la
from pr_msgs.msg import *
from manual_fit.srv import *
from copy import deepcopy
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
#~ from collision_detection.collisionHelper import *
from ik.ik import generatePlan, EvalPlan, WeightGuard, GraspingGuard, executePlanForward
from ik.helper import mat2quat, get_joints, get_params_yaml, get_tcp_pose
import goToHome
import tf
import tf.transformations as tfm
from ik.helper import Timer
from collision_free_placing import collision_free_plan, collision_free_placing, go_arc_safe

import os
import tf.transformations as tfm
    
def get_bin_corners(bin_id):
    if bin_id==0:
        term = [0,1,2,3]
    elif bin_id==1:
        term = [5,6,7,8]
    elif bin_id==2:
        term = [10,11,12,13]
    elif bin_id==3:
        term = [15,16,17,18]
    elif bin_id==4:
        term = [0,1,2,3]
    elif bin_id==5:
        term = [5,6,7,8]
    elif bin_id==6:
        term = [10,11,12,13]
    elif bin_id==7:
        term = [15,16,17,18]
    elif bin_id==8:
        term = [20,21,22,23]
    points_warped_list = []
    points_list = []
    points = []
    x_axis = np.array([1,0])
    y_axis = np.array([0,1])
    
    bin_pose = get_params_yaml('/bin'+str(bin_id)+'_pose')
    bin_center = np.asarray(bin_pose[0:2])
    b = [(1,1),(1,-1),(-1,1),(-1,-1)]
    bin_length= rospy.get_param('/bin'+str(bin_id)+'/length') 
    bin_length_2 =  bin_length/2.
    bin_width = rospy.get_param('/bin'+str(bin_id)+'/width') 
    bin_width_2 = bin_width/2. 
    for (dx, dy) in b:  
        points_tmp = bin_center + dx*x_axis*bin_length_2 + dy*y_axis*bin_width_2
        points_list.append(points_tmp.tolist())

        
    return points_list
    
def get_bin_middle():
    
    points_list = []
    for i in range(0,9):
        pose_name = '/bin'+ str(i) + '_pose/'
        params_list=['x','y']
        points = [rospy.get_param(pose_name+p) for p in params_list]
        points_list.append(points)

    return points_list
    
def get_storage_middle():
    
    points_list = []
    pose_name = '/tote_pose/'
    params_list=['x','y']
    points = [rospy.get_param(pose_name+p) for p in params_list]
    points_list.append(points)

    return points_list[0]

    
def plot_system():
    for i in range(0,9):
        #~get corners points
        points_list = get_bin_corners(i)
        points =  np.asarray(points_list)
        #~plot points
        hull = ConvexHull(points)
        plt.plot(points[:,0], points[:,1], 'o', color='b')
        for simplex in hull.simplices:
            plt.plot(points[simplex, 0], points[simplex, 1], 'b-')
        plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'b--', lw=2)
        plt.plot(points[hull.vertices[0],0], points[hull.vertices[0],1], 'bo')
        #~plot center of straight (straight)

        #~plot storage system points
    center_list = get_bin_middle()
    center_points =  np.asarray(center_list)
    storage_list = get_storage_middle()
    storage_points =  np.asarray(storage_list)
    
    plt.plot(storage_points[0], storage_points[1], 'o', color='r')
    plt.plot(center_points[:,0], center_points[:,1], 'o', color='r')
        
    plt.axis('equal')
    plt.show()
    
def gentle_move_boxes(dir_index, q_initial, pos, gripperOri_list, tip_hand_transform, listener, point_counter):
    plans = []
    axis_list = [[0,1,0], [-1,0,0], [0,-1,0], [1,0,0], [0,0,-1], [0,0,1]]
    axis = axis_list[dir_index]
    if point_counter ==4:
        bin_counter = 4
    elif point_counter ==9:
        bin_counter = 5
    elif point_counter ==14:
        bin_counter = 6
    elif point_counter ==19:
        bin_counter = 7
    elif point_counter ==24:
        bin_counter = 8

    for i in range(0,10000):
        for i in range(0,3):
            pos[i] = pos[i] + 0.002*axis[i]
        with Timer('generatePlan'):
            plan, qf, plan_possible = generatePlan(q_initial, pos, gripperOri_list, tip_hand_transform, 'fastest')
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'  
        #~execute plan
        with Timer('executePlanForward'):
            executePlanForward(plans, False, False)
        value = raw_input("Enter: Move 2mm closer to wall, s: Stop robot and record value ")
        #save data if s
        if value == 's':
            fpath = os.environ['RGRASP_BASE'] + '/catkin_ws/src/apc_config/box_corners.yaml'
            stream = open(fpath, 'r')
            data = yaml.load(stream)
            tcp_pose = get_tcp_pose(listener)
            if dir_index==4:
                terms = ['z']
            else:
                terms = ['x','y']
            for i in range(0,len(terms)):
                if dir_index==4:
                    param_name = '/bin' + str(bin_counter) + '/' + terms[i]
                    value_tmp = tcp_pose[2] - rospy.get_param('/gripper/spatula_tip_to_tcp_dist')
                else:
                    param_name = '/box_corner' + str(point_counter) + '/' + terms[i]
                    value_tmp = tcp_pose[i]
                value = value_tmp.astype(type('float', (float,), {}))
                data[param_name] = value
            with open(fpath, 'w') as yaml_file:
                yaml_file.write( yaml.dump(data, default_flow_style=False))
            break
        else:
            plans = []
    
def gentle_move(dir_index, q_initial, pos, gripperOri_list, tip_hand_transform, listener, point_counter):
    plans = []
    axis_list = [[0,1,0], [-1,0,0], [0,-1,0], [1,0,0], [0,0,-1], [0,0,1]]
    axis = axis_list[dir_index]
    if point_counter ==4:
        bin_counter = 0
    elif point_counter ==9:
        bin_counter = 1
    elif point_counter ==14:
        bin_counter = 2
    elif point_counter ==19:
        bin_counter = 3

    for i in range(0,10000):
        for i in range(0,3):
            pos[i] = pos[i] + 0.002*axis[i]
        with Timer('generatePlan'):
            plan, qf, plan_possible = generatePlan(q_initial, pos, gripperOri_list, tip_hand_transform, 'fastest')
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'  
        #~execute plan
        with Timer('executePlanForward'):
            executePlanForward(plans, False, False)
        value = raw_input("Enter: Move 2mm closer to wall, s: Stop robot and record value ")
        #save data if s
        if value == 's':
            fpath = os.environ['RGRASP_BASE'] + '/catkin_ws/src/apc_config/storage_corners.yaml'
            stream = open(fpath, 'r')
            data = yaml.load(stream)
            tcp_pose = get_tcp_pose(listener)
            if dir_index==4:
                terms = ['z']
            else:
                terms = ['x','y']
            for i in range(0,len(terms)):
                if dir_index==4:
                    param_name = '/bin' + str(bin_counter) + '/' + terms[i]
                    value_tmp = tcp_pose[2] - rospy.get_param('/gripper/spatula_tip_to_tcp_dist')
                else:
                    param_name = '/corner' + str(point_counter) + '/' + terms[i]
                    value_tmp = tcp_pose[i]
                value = value_tmp.astype(type('float', (float,), {}))
                data[param_name] = value
            with open(fpath, 'w') as yaml_file:
                yaml_file.write( yaml.dump(data, default_flow_style=False))
            break
        else:
            plans = []
            
def calibrate_boxes():
    #################################################
    #~ Generate Plans
    gripperOri = []
    gripperOri.append([-1., 0., 0., 0.])
    gripperOri.append([-0.715217411518,  -0.715217411518, 0.,0.])
    gripperOri.append([0., -1.,0., 0.])
    gripperOri.append([-0.715217411518,  0.715217411518, 0.,0.])
    #~build total list plan
    gripperOri_list = []
    ori_list = [0,0,1,2,3,3,0,0,0,0,1,2,3,0,0,0,0,0,1,2,3,0,0,0,0,0,1,2,3,0,0,0,0,0,1,2,3,0,0,0]
    for i in range(0,len(ori_list)):
        gripperOri_list.append(gripperOri[ori_list[i]])
    #~positions
    pos=[]
    #~bin4
    bin_pose = get_params_yaml('bin4_pose')
    pos.append([bin_pose[0], bin_pose[1], 0.15 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([1.1, -0.498443692923, 0.5512911677361-.15])
    pos.append([1.05, -0.599031984806, 0.5512911677361-.15])
    pos.append([1.14, -0.70, 0.5512911677361-.15])
    pos.append([1.2, -0.618239402771, 0.5512911677361-.15])
    pos.append([bin_pose[0], bin_pose[1], 0.30 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([1.08824729919, -0.808329701424, 0.5512911677361-0.05])
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    #~bin5
    bin_pose = get_params_yaml('bin5_pose')
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.79457449913, -0.498443692923, 0.5512911677361-.15])
    pos.append([0.757445454597, -0.599031984806, 0.5512911677361-.15])
    pos.append([0.79457449913, -0.70, 0.5512911677361-.15])
    pos.append([0.889182715416, -0.618239402771, 0.5512911677361-.15])
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.79457449913, -0.788900852203, 0.5512911677361-.05])
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    #~bin6
    bin_pose = get_params_yaml('bin6_pose')
    pos.append([bin_pose[0], bin_pose[1], 0.3+ rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.09962657094, -0.55, 0.539945185184+0.0])
    pos.append([0.0830265730619, -0.6, 0.539945185184+0.0])
    pos.append([0.099, -0.68, 0.539945185184+0.0])
    pos.append([0.15, -0.6, 0.539945185184+0.0])
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.145160362124, -0.846666872501, 0.65])
    pos.append([bin_pose[0], bin_pose[1], 0.4 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    #~bin7
    bin_pose = get_params_yaml('bin7_pose')
    pos.append([bin_pose[0], bin_pose[1], 0.4+ rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([-0.199928298593, -0.5, 0.539945185184+0.0])
    pos.append([-0.272910028696, -0.6, 0.539945185184+0.0])
    pos.append([ -0.195086687803, -0.75, 0.539945185184+0.0])
    pos.append([-0.100125610828, -0.6, 0.539945185184+0.0])
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([-0.203513264656, -0.86, 0.65])
    pos.append([bin_pose[0], bin_pose[1], 0.3 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    #~bin8
    bin_pose = get_params_yaml('bin8_pose')
    pos.append([bin_pose[0], bin_pose[1], 0.6+ rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([-0.09, 0.68, 0.716840863228])
    pos.append([-0.22, 0.62, 0.716840863228])
    pos.append([-0.09, 0.5, 0.716840863228])
    pos.append([0.1, 0.62, 0.716840863228])
    pos.append([bin_pose[0], bin_pose[1], 0.6 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.106256209314, 0.78, 0.8])
    pos.append([bin_pose[0], bin_pose[1], 0.6 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    
    ####################
    ## Generate Plans ##
    ####################
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    tip_hand_transform = [0.,0.,0.,0.,0.,0.]
    dir_list = [None, 0,1,2,3, None,4,None,None, 0,1,2,3, None,4,None,None, 0,1,2,3, None,4,None,None, 0,1,2,3, None,4,None,None, 0,1,2,3, None,4,None]
    point_counter = 0
    #~generate plans
    plans = []
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.5)
    goToHome.goToARC()
    goToHome.prepGripperPicking()
    ###############################
    ## Unwrap angles example use ##
    ###############################
    #~1) function to unwarp an angle to the range [-pi,pi]
    def unwrap(angle):
        if angle>np.pi:
            angle = np.mod(angle,2*np.pi)
            if angle>np.pi:
                angle = angle - 2*np.pi
        elif angle<-np.pi:
            angle = np.mod(angle,-2*np.pi)
            if angle<-np.pi:
                angle = angle + 2*np.pi
        return angle
        
    for i in range(0,16):
        plan, qf, plan_possible = generatePlan(q_initial, pos[i] , gripperOri_list[i], tip_hand_transform, 'fastest')
        if plan_possible:
            #~2) unwrap angle of joint 6 only
            plan.q_traj[-1][5] = unwrap(plan.q_traj[-1][5])
            plans.append(plan) 
            #~3) set q_initial (for next plan) with the unwrapped angle joint state
            q_initial = plan.q_traj[-1]
        else:
            return '[IK] Error'  
        #~execute plan
        executePlanForward(plans, True)
        plans = []
        if dir_list[i] is not None:
            gentle_move_boxes(dir_list[i],q_initial, pos[i], gripperOri_list[i], tip_hand_transform, listener, point_counter)
            point_counter+=1
    ##################################
          
    plans_col_free = []
    plans_col_free, q_initial, plan_possible, motion = collision_free_plan(q_initial, plans_col_free, [0,1,0,0], False, 6)
    executePlanForward(plans_col_free, True)
    
    for i in range(16,32):
    #~ for i in range(0,16):
        plan, qf, plan_possible = generatePlan(q_initial, pos[i] , gripperOri_list[i], tip_hand_transform, 'fastest')
        if plan_possible:
            plan.q_traj[-1][5] = unwrap(plan.q_traj[-1][5])
            plans.append(plan) 
            q_initial = plan.q_traj[-1]
        else:
            return '[IK] Error'  
        #~execute plan
        executePlanForward(plans, True)
        plans = []
        if dir_list[i] is not None:
            gentle_move_boxes(dir_list[i],q_initial, pos[i], gripperOri_list[i], tip_hand_transform, listener, point_counter)
            point_counter+=1
            
    plans_col_free = []
    plans_col_free, q_initial, plan_possible, motion = collision_free_plan(q_initial, plans_col_free, [0,1,0,0], False, 8)
    executePlanForward(plans_col_free, True)
            
    for i in range(32,40):
    #~ for i in range(16,24):
        plan, qf, plan_possible = generatePlan(q_initial, pos[i] , gripperOri_list[i], tip_hand_transform, 'fastest')
        if plan_possible:
            plan.q_traj[-1][5] = unwrap(plan.q_traj[-1][5])
            plans.append(plan) 
            q_initial = plan.q_traj[-1]
        else:
            return '[IK] Error'  
        #~execute plan
        executePlanForward(plans, True)
        plans = []
        if dir_list[i] is not None:
            gentle_move_boxes(dir_list[i],q_initial, pos[i], gripperOri_list[i], tip_hand_transform, listener, point_counter)
            point_counter+=1
            
def calibrate_storage():
    #################################################
    #~ Generate Plans
    gripperOri = []
    gripperOri.append([-1., 0., 0., 0.])
    gripperOri.append([-0.70710678118654757,  -0.70710678118654757, 0.,0.])
    gripperOri.append([0., -1.,0., 0.])
    gripperOri.append([-0.70710678118654757,  0.70710678118654757, 0.,0.])
    #~build total list plan
    gripperOri_list = []
    ori_list = [0,0,1,2,3,0,0,0,0,0,1,2,3,0,0,0,0,0,1,2,3,0,0,0,0,0,1,2,3,0,0,0]
    for i in range(0,len(ori_list)):
        gripperOri_list.append(gripperOri[ori_list[i]])
    #~positions
    pos=[]
    #~bin0
    bin_pose = get_params_yaml('bin0_pose')
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([1.00+.12, -0.46 + 0.19647, 0.41])
    pos.append([0.75+.12, bin_pose[1], 0.41])
    pos.append([1.+.12, -0.697281360626 + 0.19647, 0.41])
    pos.append([1.19+.12, bin_pose[1], 0.41])
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([1.00278019905+.12, -0.763054132462 + 0.19647, 0.447774887085+.05])
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    #~bin1
    bin_pose = get_params_yaml('bin1_pose')
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1+ rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([1.00+.12, -0.0943171977997 + 0.19647, 0.41])
    pos.append([0.75+.12, bin_pose[1], 0.41])
    pos.append([1.+.12, -0.330679833889 + 0.19647, 0.41])
    pos.append([1.19+.12, bin_pose[1], 0.41])
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.961158931255+.12, -0.0221027284861+ 0.19647, 0.447779148817+.05])
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    #~bin2
    bin_pose = get_params_yaml('bin2_pose')
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1+ rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([1.00+.12, 0.307756274939 + 0.19647, 0.41])
    pos.append([0.8+.12, bin_pose[1], 0.41])
    pos.append([1+.12, 0.0618322640657 + 0.19647, 0.41])
    pos.append([1.18+.12, bin_pose[1], 0.41])
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])
    pos.append([0.940047621727+.12, 0.353745698929 + 0.19647, 0.447779148817+.05])
    pos.append([bin_pose[0]+.12, bin_pose[1], 0.1 + rospy.get_param('/gripper/spatula_tip_to_tcp_dist')])

    ####################
    ## Generate Plans ##
    ####################
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    tip_hand_transform = [0.,0.,0.,0.,0.,0.]
    dir_list = [None, 0,1,2,3, None,4,None,None, 0,1,2,3, None,4,None,None, 0,1,2,3, None]
    point_counter = 0
    #~generate plans
    plans = []
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.5)
    goToHome.goToARC()
    #~ goToHome.prepGripperPicking()
    
    for i in range(0,len(pos)):
        plan, qf, plan_possible = generatePlan(q_initial, pos[i] , gripperOri_list[i], tip_hand_transform, 'fastest')
        if plan_possible:
            plans.append(plan) 
            q_initial = qf
        else:
            return '[IK] Error'  
        #~execute plan
        executePlanForward(plans, True)
        plans = []
        if dir_list[i] is not None:
            gentle_move(dir_list[i],q_initial, pos[i], gripperOri_list[i], tip_hand_transform, listener, point_counter)
            point_counter+=1
        
def generate_bins():
    point_list = [[0,1,2,3],[5,6,7,8],[10,11,12,13],[15,16,17,18]]
    z_point_list = [4,9,14,19]
    term_list = ['x','y','z']
    for i in range(0,4):
        x1 = rospy.get_param('/corner'+str(point_list[i][1])+'/x')
        x2 = rospy.get_param('/corner'+str(point_list[i][3])+'/x') + 0.001 #~frank hack to offset the asymmetry of tote
        y1 = rospy.get_param('/corner'+str(point_list[i][0])+'/y')
        y2 = rospy.get_param('/corner'+str(point_list[i][2])+'/y')
        cart_list = [(x1+x2)/2., (y1+y2)/2., rospy.get_param('/bin'+str(i)+'/z')]
        #~save bin pose to yaml
        fpath = os.environ['RGRASP_BASE'] + '/catkin_ws/src/apc_config/' +'bin'+str(i)+'_pose' +'.yaml'
        stream = open(fpath, 'r')
        data = yaml.load(stream)
        #~compute center point of tote
        for j in range(0,3):
            param_name = '/bin'+str(i)+'_pose/' + term_list[j]
            value = cart_list[j]
            #~ value = value_tmp.astype(type('float', (float,), {}))
            data[param_name] = value
        with open(fpath, 'w') as yaml_file:
                yaml_file.write( yaml.dump(data, default_flow_style=False))
                
def generate_boxes():
    point_list = [[0,1,2,3],[5,6,7,8],[10,11,12,13],[15,16,17,18],[20,21,22,23]]
    z_point_list = [4,9,14,19,24]
    bin_list = [4,5,6,7,8]
    term_list = ['x','y','z']
    for i in range(0,5):
        x1 = rospy.get_param('/box_corner'+str(point_list[i][1])+'/x')
        x2 = rospy.get_param('/box_corner'+str(point_list[i][3])+'/x') + 0.001 #~frank hack to offset the asymmetry of tote
        y1 = rospy.get_param('/box_corner'+str(point_list[i][0])+'/y')
        y2 = rospy.get_param('/box_corner'+str(point_list[i][2])+'/y')
        cart_list = [(x1+x2)/2., (y1+y2)/2., rospy.get_param('/bin'+str(bin_list[i])+'/z') + rospy.get_param('/bin'+str(bin_list[i])+'/z_flap')]
        #~save bin pose to yaml
        fpath = os.environ['RGRASP_BASE'] + '/catkin_ws/src/apc_config/' +'bin'+str(bin_list[i])+'_pose' +'.yaml'
        stream = open(fpath, 'r')
        data = yaml.load(stream)
        #~compute center point of tote
        for j in range(0,3):
            param_name = '/bin'+str(bin_list[i])+'_pose/' + term_list[j]
            value = cart_list[j]
            #~ value = value_tmp.astype(type('float', (float,), {}))
            data[param_name] = value
        with open(fpath, 'w') as yaml_file:
                yaml_file.write( yaml.dump(data, default_flow_style=False))
        
def unit_test_collision(flag = 0):
    #~ ~Build hand transform
    l1 = 0.0
    l2 = 0.0
    l3 = rospy.get_param("/gripper/calibration_to_tcp_dist")
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    gripperOriHome=[0, 1, 0, 0]
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    plans = []
    withPause = True
    #~generate plans
    #loop through sections (bins)
    counter = 0
    section_points_list = [[0,1,2,3],[0,1,2,3],[0,1,2,3],[0,2,3]]
    #~bin corners
    for i in range(0,4):
        #~get corner points from yaml
        points_warped_list, points_list = get_bin_corners(i)
        points_warped =  np.asarray(points_warped_list)
        points =  np.asarray(points_list)
        box1 = points
        box2 = points_warped
        ####################
        ## middle point ##
        ###################
        point_middle_list = get_bin_middle(i)
        point_middle = np.asarray(point_middle_list[0])
        #~transform to warped frame
        point_middle_warped = rect2quad(point_middle, box1, box2)
        #~ point_middle_warped = quad2quad(point_middle, box1, box2)
        #~ point_middle_warped = interp2d(point_middle, box1, box2)
        target_middle = np.hstack((point_middle_warped, 0.05))
        ####################
        ## corner points ##
        ###################
        for term in section_points_list[i]: 
            ##move to middle
            plan, qf, plan_possible = generatePlan(q_initial, target_middle , gripperOriHome, tip_hand_transform, 'fastest')
            counter+=1
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'  
             #~ target_position2d_warped = np.array(points_warped_list[term])
            target_position2d = np.array(points_list[term])
            #~convert to warped frame
            target_position2d = rect2quad(target_position2d,box1,box2)
            #~ target_position2d = quad2quad(target_position2d,box1,box2)
            #~ target_position2d = interp2d(target_position2d,box1,box2)
            target_position = np.hstack((target_position2d, 0.05))
            ##move to edge of structure
            target_middle2 = np.array([target_middle[0], target_position[1], target_middle[2]])
            ##move to middle edge
            plan, qf, plan_possible = generatePlan(q_initial, target_middle2 , gripperOriHome, tip_hand_transform, 'fastest')
            counter+=1
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error' 
            #~generate plan
            plan, qf, plan_possible = generatePlan(q_initial, target_position , gripperOriHome, tip_hand_transform, 'fastest')
            counter+=1
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'
            ##move to edge of structure
            plan, qf, plan_possible = generatePlan(q_initial, target_middle2 , gripperOriHome, tip_hand_transform, 'fastest')
            counter+=1
            if plan_possible:
                plans.append(plan) 
                q_initial = qf
            else:
                return '[IK] Error'

    #~Execute plans
    goToARC()
    executePlanForward(plans, withPause)
    
def unit_test_centers():
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    #~ ~Build hand transform
    l1 = 0.0
    l2 = 0.0
    l3 = rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    gripperOriHome=[0, 1, 0, 0]
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    plans = []
    withPause = True
    #~generate plans
    #loop through sections (bins)
    plans = []
    goToHome.prepGripperPicking()
    goToHome.goToARC()
    
    for i in range(0,9):
        binId = i
        collision_free_placing(binId, listener, isSuction = False)
        #~READ robot position
        rospy.sleep(.3)
        tcpPose = get_tcp_pose(listener, tcp_offset = l3) 
        tcpPose[2] = rospy.get_param('/bin'+str(i)+'_pose/z')#~- rospy.get_param('/bin'+str(i)+'/height')
        rospy.sleep(.3)
        q_initial = get_joints()
        rospy.sleep(1.0)
        ##move to edge of structure
        map_bin_id_to_ws_id = {0:0,1:1,2:2,3:3,4:0,5:0,6:4,7:4,8:5}
        plan, qf, plan_possible = generatePlan(q_initial, tcpPose[0:3] , gripperOriHome, tip_hand_transform, 'fast')
        if plan_possible:
            plans.append(plan) 
            q_initial = qf
        else:
            return '[IK] Error'
        ##move to edge of structure
        map_bin_id_to_ws_id = {0:0,1:1,2:2,3:3,4:0,5:0,6:4,7:4,8:5}
        plan, qf, plan_possible = generatePlan(q_initial,  tcpPose[0:3] - np.array([0,0,0.2]) , gripperOriHome, tip_hand_transform, 'fast', guard_on=WeightGuard(map_bin_id_to_ws_id[binId], threshold = 100))
        if plan_possible:
            plans.append(plan) 
            q_initial = qf
        else:
            return '[IK] Error'
        executePlanForward(plans, False)
        if binId>5:
            collision_free_placing(binId, listener, isSuction = False)
        plans = []

if __name__=='__main__':
    print '[Robot Calibration] Unit Test'

    rospy.init_node('robot_calibration', anonymous=True)
    #~ listener = tf.TransformListener()
    #~ br = tf.TransformBroadcaster()
    goToHome.prepGripperPicking()
    # 1)
    calibrate_storage()
    # 2)
    #~ calibrate_boxes()
    #~ go_arc_safe()
    # 3) restart pman
    # 4) 
#    generate_bins()
    # 5) 
    #~ generate_boxes()
    # 6) restart pman
    # 7) Test centers (don't do this)
    #~ unit_test_centers()
    
    

