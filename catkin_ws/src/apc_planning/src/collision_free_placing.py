#!/usr/bin/env python

from numpy.linalg import inv
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from scipy import interpolate
import tf
import tf.transformations as tfm

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
import os
import tf.transformations as tfm
from ik.ik import generatePlan, executePlanForward, executePlanBackward, fastfk
from ik.helper import get_tcp_pose, get_params_yaml, mat2quat, get_joints
from ik.ik import setAcc
from ik.roshelper import pose2list, poseTransform, pubFrame
from goToHome import goToARC


def go_arc_safe(isSuction=True):
    #########################################################
    # fhogan
    # Description:
    #~Moves robot to arc position from any position without hitting the camera posts. This function should be called after precise placing.
    #
    #~Usage
    #~ go_arc_safe(listener, isSuction=False)
    #
    #~Parameters:
    #
    #~Inputs:
    # 1) binId: Target bin identity (Integer [0-8])
    # 2) isSuction: State of the scorpion during motion, picking (False) or suction (True) configuration?. (Bool)
    #########################################################

    #~Read current robot position
    q_initial = get_joints()
    plans = []
    gripperOri=[0, 1, 0, 0]
    if isSuction:
        l1 = -rospy.get_param("/scorpion_tail/suction/x_offset")
        l2 = rospy.get_param("/scorpion_tail/suction/y_offset")
        l3 = rospy.get_param("/suction_tube/suction/length_suction_off")
        tip_hand_transform = [l1, l2, l3, 0,0,0]
    else:
        l1 = 0.
        l2 = 0.
        l3 = rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
        tip_hand_transform = [l1, l2, l3, 0,0,0]

    #~generate plan (got to right section)
    setAcc(4,4)
    plans, q_initial, plan_possible, motion = collision_free_plan(q_initial, plans, gripperOri, isSuction, 1, speed = 'superSaiyan') #~frank hack: use bin 1 (close to arc)

    #~Execute plans
    if plan_possible:
        withPause = False
        executePlanForward(plans, withPause)

    #~ go to arc position
    goToARC()

def collision_free_placing(binId, listener, obj_ID='expo_eraser', BoxBody=None,isSuction = True, with_object = False,hand_orientation=None,projected_target_position = None, isReturn = False, tip_hand_transform = None, go_faster_flag = False):

    #########################################################
    # fhogan
    # Description:
    #~Moves robot to any binId (0-8) from any position without hitting the camera post. This function should be called after precise placing to safely return to picking or suction in binId's 0-4.
    #
    #~Usage
    #~ collision_free_placing(binId=2, isSuction=False)
    #
    #~Parameters:
    #
    #~Inputs:
    # 1) binId: Target bin identity (Integer [0-8])
    # 2) isSuction: State of the scorpion during motion, picking (False) or suction (True) configuration?. (Bool)
    #########################################################
    #~Read current robot position
    go_slow = True
    # tip_hand_transform
    if tip_hand_transform == None:
        if isSuction:
            l1 = -rospy.get_param("/scorpion_tail/suction/x_offset")
            l2 = rospy.get_param("/scorpion_tail/suction/y_offset")
            l3 = rospy.get_param("/suction_tube/suction/length_suction_off")
            tip_hand_transform = [l1, l2, l3, 0,0,0]
        else:
            l1 = 0.
            l2 = 0.
            l3 = rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
            tip_hand_transform = [l1, l2, l3, 0,0,0]
        
    lowest_bound = 0.30
    highest_bound = 0.65  # set a highest bound to avoid plan failure.
    
    
    # pre_basepose and bin_pose
    
    pre_basepose = poseTransform(tip_hand_transform, "link_6","map", listener)  
    bin_pose = get_params_yaml('bin'+str(binId)+'_pose')
    if binId==2:
        bin_pose[2] = get_params_yaml('bin1_pose')[2]
    
    # setting gripperOri
    
    if hand_orientation is not None:
        gripperOri = hand_orientation
    else:
        gripperOri=[0, 1, 0, 0]
    rospy.logdebug('[Collision_free_moving] gripperOri = %s',gripperOri)
    
    # setting placing_height
    if with_object:
        if BoxBody is not None:
            z_list = []
            for i in range(0,8):
                box_pos_tmp = poseTransform([BoxBody[i][0], BoxBody[i][1], BoxBody[i][2],0.0,0.0,0.0,1.0], "link_6","map", listener)
                z_list.append(box_pos_tmp[2])
            height_given_by_vision = pre_basepose[2] - min(z_list)
        else:
            height_given_by_vision=0

        # height of suction_cup depending on object size & be bounded
        try:
            obj_dim = rospy.get_param('obj')[str(obj_ID)]
            obj_max_dim = np.linalg.norm(obj_dim['dimensions'])
            go_slow = obj_dim['slow_move']   # check it with Eudald
            if obj_dim['type'] == 'book':
                obj_max_dim = obj_max_dim*2
        except:
            obj_max_dim = highest_bound
        placing_height = max(obj_max_dim,lowest_bound,height_given_by_vision*1.1)
    else:
        placing_height = lowest_bound
    if is_in_bin(listener = listener,bin_id=3) and binId < 5:   #if initially the robot is in bin3, rise hight since the bin is higher
        bin_pose[2] = max(get_params_yaml('bin3_pose')[2],bin_pose[2])
    #~ elif is_in_bin(listener = listener,bin_id=3) and bin_Id < 5:   #if initially the robot is in bin3, rise hight since the bin is higher
        #~ bin_pose[2] = max(get_params_yaml('bin5_pose')[2],bin_pose[2])
    placing_height = min(placing_height,highest_bound)
    
    target_pos1_z = bin_pose[2]+placing_height
    

    #~generate plan (got to right section)
    q_initial = get_joints()
    plans = []
    plans, q_initial, plan_possible, motion = collision_free_plan(q_initial, plans, gripperOri, isSuction, binId)

    # IF it's in bin already, the target_pos1_z height should not be higher than pre_basepose[2]
    if is_in_bin(listener = listener,bin_id=binId) and target_pos1_z > pre_basepose[2]:
        lowest_bound = 0.15
        target_pos1_z = max(pre_basepose[2],lowest_bound)
        
    #~setting target positions
    target_pos1 = bin_pose[0:3]
    if projected_target_position is not None:
        target_pos1[0:2] = projected_target_position[0:2]

        

    if motion ==6:
        target_pos1_z = min(target_pos1_z,highest_bound)
        target_pos0 = np.array([pre_basepose[0], pre_basepose[1],target_pos1_z])
        target_pos1[2] = target_pos1_z
        #~add weightpoint
        ipos = interp_pos(target_pos0,target_pos1,.5)
        target_pos_list = [target_pos0,ipos,target_pos1]
    else:
        target_pos1[2] = bin_pose[2] + 0.15
        target_pos_list = [target_pos1]
        
    rospy.loginfo('[Collision Free Motion] Going to target_pos_list: %s', target_pos_list)
    #~generate plan (got to right binId)
    
    for target_pos in target_pos_list:
        this_plan_possible = False
        while not this_plan_possible and target_pos[2] > lowest_bound:
            plan, qf, this_plan_possible = generatePlan(q_initial, target_pos, gripperOri, tip_hand_transform, 'fastest')
            if this_plan_possible:
                plans.append(plan)
                q_initial = qf
                rospy.loginfo('[Collision Free Motion] Going to target_pos: %s', target_pos)
                if with_object:
                    if go_faster_flag and not go_slow:
                        rospy.logwarn("[Suction] go_faster_flag is True!! Go yolo~")
                        plan.setSpeedByName('yolo')
                    else:
                        plan.setSpeedByName('faster')
                else:
                    plan.setSpeedByName('superSaiyan')
                break
            else:
                generatePlan(q_initial, target_pos, gripperOri, tip_hand_transform, 'fastest',plan_name='collision_free_motion')
                target_pos[2] = target_pos[2] - 0.05
                rospy.logwarn( '[Collision Free Motion] Plan not possible, change target_pos[2] to %s.', target_pos[2])
                
        plan_possible = plan_possible and this_plan_possible
        if not plan_possible:
            rospy.logerr('[Collision Free Motion] All plan failed or no need to plan!: %s', target_pos)
            #~ print '[Collision Free Motion] All plan failed.', target_pos[2]
            break
        
    #~Execute plans
    if len(plans) != 0:
        withPause = False
        if with_object:
            setAcc(1,1)
        else:
            setAcc(4,4)
        executePlanForward(plans, withPause)
        
    if isReturn:
        return q_initial
        
def is_in_bin(listener,bin_id=0):
    l3 = rospy.get_param("/gripper/length")
    tip_hand_transform = [0, 0, l3, 0,0,0]
    pre_basepose = poseTransform(tip_hand_transform, "link_6","map", listener)

    bin_pose = rospy.get_param('bin' + str(bin_id) + '_pose') #TODO: Modify the pose as you want. It is a dictionary:{qw:,qx:,qy:,qz:,x:,y:,z:}
    bin_dimensions = rospy.get_param('bin'+ str(bin_id)) #{height:, width:, length: }
    dx = bin_dimensions['length']
    dy = bin_dimensions['width']
    dz = bin_dimensions['height']
    margin = 0.02
    if abs(pre_basepose[0] - bin_pose['x']) < dx/2-margin:
        if abs(pre_basepose[1] - bin_pose['y']) < dy/2-margin:
            return True
    return False
    
def interp_pos(pos1,pos2,alpha):
    pos1=np.array(pos1)
    pos2=np.array(pos2)
    return (1-alpha)*pos1+alpha*pos2

def collision_free_plan(q_initial, plans, gripperOri, isSuction, binId, speed = 'fastest'):
    #########################################################
    # fhogan
    # Description:
    #~Appends list plans with new plan that generates a motion from any robot position to the desired box without hitting the camera bar.
    #
    #~Usage
    #~ plans, q_initial, plan_possible, motion = collision_free_plan(q_initial, plans, gripperOri, isSuction, binId)
    #
    #~Parameters:
    #
    #~Inputs:
    # 1) q_initial: Latest joint pose list of robot
    # 2) plans: List of robot plans (global list of all plans).
    # 3) gripperOri: Desired quaternion of gripper.
    # 4) isSuction: State of the scorpion during motion, picking (False) or suction (True) configuration?. (Bool)
    # 5) binId: Target bin identity (Integer [0-8])
    #
    #~Outputs:
    #~1) plans: List of robot plans (global list of all plans appened with new collision free plan).
    #~2) q_initial: Joint pose to be used when building next plan.
    #~3) plan_possible: Was IK feasible? (Bool)
    #~4) motion: Descriptor of the motion. If motion==6, then the commanded motion was from bins 0-5 to bins 0-5.
    #########################################################
    #~Initialize variables
    bin_list = get_bin_list()
    tip_hand_transform = [0, 0, 0, 0,0,0]
    ###########################################
    ## generate plans for each int. position ##
    ###########################################
    pos = []
    pos_tot = []
    pos_tot.append(np.array([941.66,-440.64,1100.81])/1000.)
    pos_tot.append(np.array([465.64,-713.0,1253.81])/1000.)
    pos_tot.append(np.array([-123.08, -812.62, 1174.66])/1000.)
    pos_tot.append(np.array([941.66,440.64,1100.81])/1000.)
    pos_tot.append(np.array([488.64,503.0,1370.81])/1000.)
    pos_tot.append(np.array([-60.7, 645.96, 1150.66])/1000.)

    isExecute, motion = check_safe_motion(q_initial, binId)
    if isExecute:
        motion_list_right = [1,3]
        motion_list_left = [0,2]
        #~if going to bins 6,7
        if motion == 0:
            print '[Collision Free Placing] Goind for bins 6,7'
            for i in [0,1,2]:
                pos.append(pos_tot[i])
        if motion == 1:
            print '[Collision Free Placing] Goind for bins 8'
            for i in [3,4,5]:
                pos.append(pos_tot[i])
        if motion == 2:
            print '[Collision Free Placing] Goind for bins 0,1,2,3,4 from bins 6,7'
            for i in [2,1,0]:
                pos.append(pos_tot[i])
        if motion == 3:
            print '[Collision Free Placing] Goind for bins 0,1,2,3,4 from bins 6,7'
            for i in [5,4,3]:
                pos.append(pos_tot[i])
        if motion == 4:
            print '[Collision Free Placing] Goind for bins 8 from bins 6,7'
            for i in [2,1,0,3,4,5]:
                pos.append(pos_tot[i])
        if motion == 5:
            print '[Collision Free Placing] Goind for bins 8 from bins 6,7'
            for i in [5,4,3,0,1,2]:
                pos.append(pos_tot[i])
        #~ print '[Collision Free Placing] pos', pos

        #################################################
        ## Convert hand transform for suction/grasping ##
        #################################################

        z_delta = rospy.get_param('/gripper/suction_tip_to_tcp_dist') - rospy.get_param('/gripper/spatula_tip_to_tcp_dist')
        if isSuction is False:
            for i in range(0, len(pos)):
                pos[i][2] = pos[i][2] - z_delta
        #~build plans
        for i in range(0, len(pos)):
            print '[Collision Free Placing] pos', pos[i]
            plan, qf, plan_possible = generatePlan(q_initial, pos[i], gripperOri, tip_hand_transform, speed)
            if plan_possible:
                plans.append(plan)
                q_initial = qf
            else:
                return '[IK] Error'
    else:
        print '[Collision Free Placing] Already in correct section. No plans generated.'
        qf = q_initial
        plan_possible = True
        return plans, qf, plan_possible, motion

    #~return plan, final joint state qf, and if plan was possible
    return plans, qf, plan_possible, motion

def check_safe_motion(q_initial, binId):
    bin_list = get_bin_list()
    tcp_pos = fastfk(q_initial)

    isExecute = False
    #~find section
    if tcp_pos[0]> 0.4:
        section = 0
    elif tcp_pos[0]< 0.4 and tcp_pos[1]< 0.0:
        section = 1
    else:
        section = 2
    print '[Collision Free Placing] Currently in section', section
    #~check if desired motion is safe
    motion = None
    #~moving from section 0 to section 1
    if (binId  in bin_list[1]) and (section == 0):
        isExecute = True
        motion = 0
    #~moving from section 0 to section 2
    if (binId  in bin_list[2]) and (section == 0):
        isExecute = True
        motion = 1
    #~moving from section 1 to section 0
    if (binId  in bin_list[0]) and (section == 1):
        isExecute = True
        motion = 2
    #~moving from section 2 to section 0
    if (binId  in bin_list[0]) and (section == 2):
        isExecute = True
        motion = 3
    #~moving from section 1 to section 2
    if (binId  in bin_list[2]) and (section == 1):
        isExecute = True
        motion = 4
    #~moving from section 2 to section 1
    if (binId  in bin_list[1]) and (section == 2):
        isExecute = True
        motion = 5
    #~moving from section 0 to section 0
    if (binId  in bin_list[0]) and (section == 0):
        motion = 6
    print '[Collision Free Placing] Going for motion', motion
    return isExecute, motion

def get_bin_list():
    bin_list = []
    bin_list.append([0,1,2,3,4,5])
    bin_list.append([6,7])
    bin_list.append([8])
    return bin_list


def unit_test():
    #~ ~Build hand transform
    l1 = 0.0
    l2 = 0.0
    l3 = 0.0
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    gripperOriHome=[0, 1, 0, 0]
    q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    plans = []
    withPause = True
    #~generate plans
    ######################
    ## go active vision ##
    ######################
    for binId in range(0,9):
        plans = []
        #~generate plan
        plans, qf, plan_possible, motion = collision_free_plan(q_initial, plans, gripperOriHome, True, binId)
        executePlanForward(plans, False)


if __name__=='__main__':
    print '[Collision Free Placing]'

    rospy.init_node('collision_free_placing', anonymous=True)
    #~ unit_test_totes(binId = 6, flag=0)
    #~ collision_free_placing(flag=0)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    collision_free_placing(8, listener, obj_ID='expo_eraser', BoxBody=None,isSuction = False,with_object = True,hand_orientation=None,isReturn = False, tip_hand_transform = None)
    #~ unit_test()


