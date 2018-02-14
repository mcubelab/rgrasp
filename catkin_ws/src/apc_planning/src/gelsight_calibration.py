#!/usr/bin/env python

import rospy, sys
import numpy as np
#from manual_fit.srv import *
from copy import deepcopy
import roslib; roslib.load_manifest("robot_comm")
#from robot_comm.srv import *
import tf
#from collision_free_placing import  collision_free_placing
from ik.ik import generatePlan, EvalPlan, WeightGuard, executePlanForward
#from ik.helper import get_joints, mat2quat, get_params_yaml, reference_frames, drop_pose_transform
from collision_detection.collisionHelper import collisionFree
import gripper
import ik.helper
import ik.roshelper
import ik.visualize_helper
import sensor_msgs.msg
import std_msgs.msg
import goToHome
from visualization_msgs.msg import MarkerArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
#from grasp_data_recorder import GraspDataRecorder
import datetime
try:
    import lasers
except:
    pass


def calibrate_gelsight(isExecute=True, withPause=False):
    arc_ori = np.array([0,1,0,0])
    ori_1 = np.array([ -0.70710678118654757, 0.70710678118654757, 0, 0])
    ori_2 = np.array([ 0.70710678118654757, 0.70710678118654757, 0, 0])
    #initialize Parameters
    tip_hand_transform = [0,0,0, 0,0,0]
    #read initial robot poses
    q_initial = ik.helper.get_joints()
    #go to arc_1
    # goToHome.goToARC()
    plans_list = []
    pose_list = []
    #arc position (high)
    pose_list.append(np.array([1.0,0.,0.792, 0,1,0,0]))
    pose_list.append(np.array([.642, .72005, .95168, .69934, .70796, -0.06744, .07189]))
    pose_list.append(np.array([.642, .72005, .75559, .69934, .70796, -0.06744, .07189]))
    pose_list.append(np.array([.56577, .72005, .75559, .69934, .70796, -0.06744, .07189]))
    pose_list.append(np.array([.48138, .72005, .75559, .69934, .70796, -0.06744, .07189]))
    pose_list.append(np.array([.48138, .72005, .95168, .69934, .70796, -0.06744, .07189]))
    pose_list.append(np.array([1.0,0.,0.792, 0,1,0,0]))
    speed_list = ['superSaiyan', 'superSaiyan', 'slow', 'slow', 'slow', 'slow', 'superSaiyan']
    #gripper hand_commands
    # is_gripper_list = [0,0,0,0,0,0,0]
    is_gripper_list = [0,0,1,1,1,0,0]
   #~1. Open gripper
    grasp_plan = EvalPlan('helper.moveGripper(%f, 200)' % 0.11)
    plans_list.append(grasp_plan)

    #2. generate robot motions
    for i in range(len(pose_list)):
        print ' speed_list[i]',  speed_list[i]
        plan, qf, plan_possible = generatePlan(q_initial, pose_list[i][0:3], pose_list[i][3:7], tip_hand_transform, speed_list[i], plan_name = str(i))
        if plan_possible:
            plans_list.append(plan)
            q_initial = qf
        else:
            print ('[Release Safe] Plans failed:')
        if is_gripper_list[i]:
            #sleep
            sleep_plan=EvalPlan('rospy.sleep(1.0)')
            plans_list.append(sleep_plan)
            #close gripper
            grasp_plan = EvalPlan('helper.graspinGripper(%f,%f)'%(800,30))
            plans_list.append(grasp_plan)
            #sleep
            sleep_plan=EvalPlan('rospy.sleep(1.5)')
            plans_list.append(sleep_plan)
            #capture image

            gelsight_plan = EvalPlan('helper.capture_gelsight()')
            plans_list.append(gelsight_plan)
            #sleep
            sleep_plan=EvalPlan('rospy.sleep(.5)')
            plans_list.append(sleep_plan)
            #open gripper
            grasp_plan = EvalPlan('helper.moveGripper(%f, 200)' % 0.11)
            plans_list.append(grasp_plan)
            #sleep
            sleep_plan=EvalPlan('rospy.sleep(2.)')
            plans_list.append(sleep_plan)

    #execute plan_name
    if isExecute and plan_possible:
        executePlanForward(plans_list, withPause)

# To test the function
if __name__=='__main__':
    rospy.init_node('gelsight_calibration', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    calibrate_gelsight()
