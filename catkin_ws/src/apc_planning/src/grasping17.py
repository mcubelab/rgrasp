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
from visualization_msgs.msg import MarkerArray
#from grasp_data_recorder import GraspDataRecorder
import datetime
try:
    import lasers
except:
    pass

def check_collision(tcp_pose, delta_pos, listener, br, binId=0):
    #new positions
    tcp_pose[0:3] = tcp_pose[0:3] + delta_pos
    #~Define reference frames
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = ik.helper.reference_frames(listener= listener, br=br)
    #~get grasp pose and gripper opening from vision
    graspPos, hand_X, hand_Y, hand_Z, grasp_width = ik.helper.get_picking_params_from_7(tcp_pose, None, listener, br)
    #~build gripper orientation matrix 3x3
    hand_orient_norm = np.vstack([hand_X,hand_Y,hand_Z])
    hand_orient_norm=hand_orient_norm.transpose()
    #~Grasp relocation script
    hand_orient_quat=ik.helper.mat2quat(hand_orient_norm)
    euler = tf.transformations.euler_from_quaternion(hand_orient_quat)
    theta = euler[2] - np.pi
    is_collision = collisionFree(graspPos, binId=binId, listener=listener, br=br, finger_opening = grasp_width, safety_margin = 0.00, theta = theta, is_bool = True)
    return is_collision

def grasp_correction(objInput,
                        delta_pos,
                        listener,
                        br,
                        isExecute = True,
                        objId = 'expo_eraser',
                        binId=0,
                        withPause = False,
                        viz_pub = None,
                        recorder = None):

    #define constants
    spatula_tip_to_tcp_dist = rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    # l1 = 0.0
    # l2 = 0.0
    # l3 = spatula_tip_to_tcp_dist
    # tip_hand_transform = [l1, l2, l3, 0,0,0]
    # #get current robot configuration
    tcpPose = ik.helper.get_tcp_pose(listener, tcp_offset = spatula_tip_to_tcp_dist)
    delta_pose_hand = np.zeros((7))
    delta_pose_hand[0:3] = delta_pos + np.array([0,0,spatula_tip_to_tcp_dist])
    delta_pose_hand[3:7] = np.array([0,0,0,1])
    pose_world = ik.roshelper.poseTransform(delta_pose_hand, "link_6", "map", listener)
    # #make sure poses are array types
    # targetPose = np.array(targetPose)
    # delta_pos = np.array(delta_pos)
    # #compute new grasp location
    # objInput_tmp = np.zeros((7))
    # objInput_tmp[0:3] = targetPose[0:3] + delta_pos
    # objInput_tmp[3:7] = targetPose[3:7]
    #delta_pos is in hand_frame
    objInput_arr = np.array(objInput)
    objInput_arr[0:3] = pose_world[0:3]
    #release grasp safely
    release_safe(listener)
    #go for new position
    return grasp(objInput_arr,
              listener,
              br,
              isExecute = True,
              objId = 'expo_eraser',
              binId=0,
              withPause = False,
              viz_pub = None,
              recorder = None)

def release_safe(listener, isExecute=True, withPause=False):
    plans = []
    #Initialize parameters
    spatula_tip_to_tcp_dist = rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    bin_pose = ik.helper.get_params_yaml('bin'+str(0)+'_pose')
    UpdistFromBinFast = 0.15
    #Initial configuration of robot
    q_initial = ik.helper.get_joints()
    targetPose = ik.helper.get_tcp_pose(listener, tcp_offset = spatula_tip_to_tcp_dist)
    #Desired new safe position above binNum
    targetPose[2] = bin_pose[2] + UpdistFromBinFast
    #################
    ## Build plans ##
    #################
   #~1. Open gripper
    grasp_plan = EvalPlan('helper.moveGripper(%f, 200)' % 0.11)
    plans.append(grasp_plan)
    #~2. Open spatula
    grasp_plan = EvalPlan('spatula.open()')
    plans.append(grasp_plan)
    #~3. Move up
    plan, qf, plan_possible = generatePlan(q_initial, targetPose[0:3], targetPose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'go_safe_bin')
    if plan_possible:
        plans.append(plan)
        q_initial = qf
    else:
        print ('[Release Safe] Plans failed:')

    #execute plan_name
    if isExecute and plan_possible:
        executePlanForward(plans, withPause)

def grasp(objInput,
          listener,
          br,
          isExecute = True,
          objId = 'expo_eraser',
          binId=0,
          withPause = False,
          viz_pub = None,
          recorder = None):

    #########################################################
    # fhogan and nikhilcd
    # Description:
    #~2017 picking primitive
    #
    #~Usage
    #~ pick(objInput, listener, br, isExecute = True, objId = 'cheezit_big_original', flag = 1, withPause = False, rel_pose=None, BoxBody=None, place_pose=None, viz_pub=None):
    #
    #~Parameters:
    #
    #~Inputs:
    # 1) objInput: Input from vision or RViz (list of 12, or 7)
    # 2) listener: Input from vision or RViz (list of 12, or 7)
    # 3) br: Input from vision or RViz (list of 12, or 7)
    # 4) isExecute: execute the robot motion if possible or not? (Bool)
    # 5) objId: name of object (str)
    # 6) binId: bin in which we need to place the objects. (It is useless when flag 0 and 1)
    # 7) flag: Type of action directed by planning (0:run picking, 1:go to home (camera), 2: drop the object at target location)
    # 8) withPause: Pause between robot motions (Bool)
    # 9) rel_pose: object pose relative to link_6
    # 10) BoxBody: Bounding box points resolved in link 6 frame
    # 11) place_pose: desired position of bounding box
    # 12) viz_pub: desired position of bounding box
    #
    #~Output:
    # Dictionary of following keys:
    # 1) grasp_possible: Does the object fit in the hand? (True/False)
    # 2) plan_possible: Has the plan succeeded? (True/False)
    # 3) execution_possible: Is there anything in the grasp? (True/False)
    # 4) gripper_opening: gripper opening after grasp attempt (in meters)
    # 5) graspPose: pose of the gripper at grasp (7X1 - position and quaternion)
    # 6) gelsight_data: [] , dummy for now
    #########################################################

    #rospy.logdebug(msg, *args)
    #rospy.loginfo(msg, *args)
    #rospy.logwarn(msg, *args)
    #rospy.logerr(msg, *args)
    #rospy.logfatal(msg, *args)

    ###############################
    ## Pring input variables for ##
    ###############################
    rospy.logdebug('[Picking] objInput %s', objInput)
    rospy.logdebug('[Picking] objId %s', objId)
    rospy.logdebug('[Picking] withPause %s', withPause)

    ########################
    ## Initialize values ##
    ########################
    q_initial = ik.helper.get_joints()
    bin_pose = ik.helper.get_params_yaml('bin'+str(binId)+'_pose')
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    delta_vision_pose = ik.helper.get_params_yaml('vision_pose_picking')
    vision_pos=np.array(bin_pose[0:3])+np.array(delta_vision_pose[0:3])
    plans = []
    plans_grasping = []
    plans_grasping2 = []
    plans_guarded = []
    plans_guarded2 = []
    plans_placing = []
    plans_placing2 = []
    graspPose=[]
    plan_possible = False
    execution_possible = False
    gripper_opening = 0.0
    grasp_possible = True
    gelsight_data=[]
    collision = False
    final_object_pose=None
    rospy.set_param('is_record', False)
    rospy.set_param('is_contact', False)

    def compose_output():
        return {'collision':collision,'grasp_possible':grasp_possible,'plan_possible':plan_possible,'execution_possible':execution_possible,'gripper_opening':gripper_opening,'graspPose':graspPose,'gelsight_data':gelsight_data,'final_object_pose':final_object_pose}

    #########################
    ## Constant parameters ##
    #########################
    UpdistFromBinFast = 0.15 # Margin above object during "move to a location"
    UpdistFromBinGuarded = 0.05 # Margin above object during "move to a location"
    gripperOriHome=[0, 1, 0, 0]
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    vision_pos[2] = 0.17786528

    ###############################
    ## Picking primitive  ##
    ###############################
#        ik.visualize_helper.visualize_grasping_proposals(viz_pub, np.asarray([objInput]),  listener, br, False)
    #~Define reference frames
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = ik.helper.reference_frames(listener= listener, br=br)

    #~get grasp pose and gripper opening from vision
    if len(objInput)==12:
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = ik.helper.get_picking_params_from_12(objInput)
        graspPos = graspPos # + hand_X*0.02*1
    elif len(objInput)==7:
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = ik.helper.get_picking_params_from_7(objInput, objId, listener, br)

    #frank hack for stationary spatula:
    grasp_width = 0.11
    #~build gripper orientation matrix 3x3
    hand_orient_norm = np.vstack([hand_X,hand_Y,hand_Z])
    hand_orient_norm=hand_orient_norm.transpose()

    #~Grasp relocation script
    hand_orient_quat=ik.helper.mat2quat(hand_orient_norm)
    euler = tf.transformations.euler_from_quaternion(hand_orient_quat)
    theta = euler[2] - np.pi
    colFreePose = collisionFree(graspPos, binId=binId, listener=listener, br=br, finger_opening = grasp_width, safety_margin = 0.00, theta = theta)
    graspPos[0:2] = colFreePose[0:2]

        #################################
        ## GENERATE PLANS OF PRIMITIVE ##
        #################################.
        #~ Start from home position
#        if isExecute:
#            q_initial = collision_free_placing(binId, listener, isSuction = False, with_object = False, hand_orientation=[0,1,0,0], projected_target_position = bin_pose[0:3], isReturn = True)
#            scorpion.back()

    #~0. Move to a location above the bin, Rotate gripper to grasp orientation
    pregrasp_targetPosition = graspPos
    pregrasp_targetPosition[2] = bin_pose[2] + UpdistFromBinFast
    plan, qf, plan_possible = generatePlan(q_initial, pregrasp_targetPosition, hand_orient_quat, tip_hand_transform, 'superSaiyan', plan_name = 'go_safe_bin')
    if plan_possible:
        plans.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~1. Move to to surface of bin
    pregrasp_targetPosition[2] = bin_pose[2] + UpdistFromBinGuarded
    plan, qf, plan_possible = generatePlan(q_initial, pregrasp_targetPosition, hand_orient_quat, tip_hand_transform, 'superSaiyan', plan_name = 'go_top_bin')
    if plan_possible:
        plans.append(plan)
        q_initial = qf
    else:
        return compose_output()

   #~2. Open gripper
    grasp_plan = EvalPlan('helper.moveGripper(%f, 200)' % grasp_width)
    plans.append(grasp_plan)

    #~ 3. Open spatula
    grasp_plan = EvalPlan('spatula.open()')
    plans.append(grasp_plan)

    grasp_targetPosition=deepcopy(graspPos)
#        rospy.loginfo('[Picking] Grasp Target %s', grasp_targetPosition)

    #~4. sleep
    sleep_plan = EvalPlan('rospy.sleep(0.2)')
    plans.append(sleep_plan)

     #~5. perform guarded move down
    grasp_targetPosition[2] = bin_pose[2] -  rospy.get_param('/tote/height') + 0.01  #~frank hack for safety

    plan, qf, plan_possible = generatePlan(q_initial, grasp_targetPosition, hand_orient_quat, tip_hand_transform, 'faster', guard_on=WeightGuard(binId, threshold = 100), plan_name = 'guarded_pick')
    if plan_possible:
        plans_guarded.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~7. Close spatula
    grasp_plan = EvalPlan('spatula.close()')
    plans_guarded2.append(grasp_plan)

    #~8. Close gripper
    grasp_plan = EvalPlan('helper.graspinGripper(%f,%f)'%(800,30))
    plans_guarded2.append(grasp_plan)

    #~9. sleep
    sleep_plan = EvalPlan('rospy.sleep(0.6)')
    plans_guarded2.append(sleep_plan)

    #~10. Move to a location above the bin
    # plan, qf, plan_possible = generatePlan(q_initial, vision_pos, delta_vision_pose[3:7], tip_hand_transform, 'fastest', plan_name = 'retrieve_object')
    # if plan_possible:
    #     plans_grasping.append(plan)
    #     q_initial = qf
    # else:
    #     return compose_output()

    ################
    ## EXECUTION ###
    ################

    if isExecute and plan_possible:

        executePlanForward(plans, withPause)

        ###We start recording now
        if recorder is not None:
            lasers.start(binId)
            recorder.start_recording(action='grasping', action_id=str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")), tote_num=binId, frame_rate_ratio=5, image_size=-1)

        #Execute guarded move
        rospy.set_param('is_record', True)
        executePlanForward(plans_guarded, withPause)
        if rospy.get_param('is_contact'):
            ik.helper.move_cart(dz=0.005)
        executePlanForward(plans_guarded2, withPause)
        rospy.set_param('is_record', False)

    elif not isExecute:
      execution_possible=plan_possible

    return compose_output()

def retrieve(listener,
              br,
              isExecute = True,
              objId = 'expo_eraser',
              binId=0,
              withPause = False,
              viz_pub = None,
              recorder = None):

    liftoff_pub = rospy.Publisher('/liftoff_time', std_msgs.msg.String, queue_size = 10, latch=False)

    def compose_output():
        return {'collision':collision,'grasp_possible':grasp_possible,'plan_possible':plan_possible,'execution_possible':execution_possible,'gripper_opening':gripper_opening,'graspPose':graspPose,'gelsight_data':gelsight_data,'final_object_pose':final_object_pose}

    q_initial = ik.helper.get_joints()
    bin_pose = ik.helper.get_params_yaml('bin'+str(binId)+'_pose')
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    delta_vision_pose = ik.helper.get_params_yaml('vision_pose_picking')
    vision_pos=np.array(bin_pose[0:3])+np.array(delta_vision_pose[0:3])
    vision_pos[2] = 0.17786528
    plans = []
    plans_grasping = []
    graspPose=[]
    plan_possible = False
    execution_possible = False
    gripper_opening = 0.0
    grasp_possible = True
    gelsight_data=[]
    collision = False
    final_object_pose=None
    rospy.set_param('is_record', False)
    rospy.set_param('is_contact', False)

    # ~10. Move to a location above the bin
    plan, qf, plan_possible = generatePlan(q_initial, vision_pos, delta_vision_pose[3:7], tip_hand_transform, 'fastest', plan_name = 'retrieve_object')
    if plan_possible:
        plans_grasping.append(plan)
        q_initial = qf
    else:
        return compose_output()

    if isExecute and plan_possible:
      #Publish liftoff time
      liftoff_msgs = std_msgs.msg.String()
      liftoff_msgs.data = 'Liftoff (Robot command)'
      liftoff_pub.publish(liftoff_msgs)

      #Execute non-guarded grasp plan move
      executePlanForward(plans_grasping, withPause)

      ###We stop recording
      if recorder is not None:
          recorder.stop_recording(save_action=True)
          lasers.stop(binId)

    #~Check if picking success
    low_threshold = 0.0035
    high_threshold = 0.015

    if isExecute and plan_possible:
      rospy.sleep(2)
      gripper_opening=gripper.getGripperopening()
      if gripper_opening > high_threshold:
          rospy.loginfo('[Picking] ***************')
          rospy.loginfo('[Picking] Pick Successful (Gripper Opening Test)')
          rospy.loginfo('[Picking] ***************')
          execution_possible = None #temporary hack
      else:
          rospy.loginfo('[Picking] ***************')
          rospy.loginfo( '[Picking] Pick Inconclusive (Gripper Opening Test)')
          rospy.loginfo( '[Picking] ***************')
          execution_possible = None

    elif not isExecute:
      execution_possible=plan_possible

    return compose_output()


def place(listener,
          br,
          isExecute = True,
          binId=0,
          withPause = False,
          rel_pose = None,
          BoxBody = None,
          place_pose = None,
          viz_pub = None,
          is_drop = True,
          recorder = None):

    #########################################################
    # fhogan and nikhilcd
    # Description:
    #~2017 picking primitive
    #
    #~Usage
    #~ pick(objInput, listener, br, isExecute = True, objId = 'cheezit_big_original', flag = 1, withPause = False, rel_pose=None, BoxBody=None, place_pose=None, viz_pub=None):
    #
    #~Parameters:
    #
    #~Inputs:
    # 1) objInput: Input from vision or RViz (list of 12, or 7)
    # 2) listener: Input from vision or RViz (list of 12, or 7)
    # 3) br: Input from vision or RViz (list of 12, or 7)
    # 4) isExecute: execute the robot motion if possible or not? (Bool)
    # 5) objId: name of object (str)
    # 6) binId: bin in which we need to place the objects. (It is useless when flag 0 and 1)
    # 7) flag: Type of action directed by planning (0:run picking, 1:go to home (camera), 2: drop the object at target location)
    # 8) withPause: Pause between robot motions (Bool)
    # 9) rel_pose: object pose relative to link_6
    # 10) BoxBody: Bounding box points resolved in link 6 frame
    # 11) place_pose: desired position of bounding box
    # 12) viz_pub: desired position of bounding box
    #
    #~Output:
    # Dictionary of following keys:
    # 1) grasp_possible: Does the object fit in the hand? (True/False)
    # 2) plan_possible: Has the plan succeeded? (True/False)
    # 3) execution_possible: Is there anything in the grasp? (True/False)
    # 4) gripper_opening: gripper opening after grasp attempt (in meters)
    # 5) graspPose: pose of the gripper at grasp (7X1 - position and quaternion)
    # 6) gelsight_data: [] , dummy for now
    #########################################################

    #rospy.logdebug(msg, *args)
    #rospy.loginfo(msg, *args)
    #rospy.logwarn(msg, *args)
    #rospy.logerr(msg, *args)
    #rospy.logfatal(msg, *args)

    ###############################
    ## Pring input variables for ##
    ###############################
    rospy.logdebug('[Picking] withPause %s', withPause)
    rospy.logdebug('[Picking] rel_pose %s', rel_pose)
    rospy.logdebug('[Picking] BoxBody %s', BoxBody)
    rospy.logdebug('[Picking] place_pose %s', place_pose)

    ########################
    ## Initialize values ##
    ########################
    q_initial = ik.helper.get_joints()
    bin_pose = ik.helper.get_params_yaml('bin'+str(binId)+'_pose')
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    delta_vision_pose = ik.helper.get_params_yaml('vision_pose_picking')
    vision_pos=np.array(bin_pose[0:3])+np.array(delta_vision_pose[0:3])
    plans = []
    plans_grasping = []
    plans_grasping2 = []
    plans_guarded = []
    plans_guarded2 = []
    plans_placing = []
    plans_placing2 = []
    graspPose=[]
    plan_possible = False
    execution_possible = False
    gripper_opening = 0.0
    grasp_possible = True
    gelsight_data=[]
    collision = False
    final_object_pose=None
    rospy.set_param('is_record', False)
    rospy.set_param('is_contact', False)


    liftoff_pub = rospy.Publisher('/liftoff_time', std_msgs.msg.String, queue_size = 10, latch=False)

    def compose_output():
        return {'collision':collision,'grasp_possible':grasp_possible,'plan_possible':plan_possible,'execution_possible':execution_possible,'gripper_opening':gripper_opening,'graspPose':graspPose,'gelsight_data':gelsight_data,'final_object_pose':final_object_pose}

    #########################
    ## Constant parameters ##
    #########################
    UpdistFromBinFast = 0.15 # Margin above object during "move to a location"
    UpdistFromBinGuarded = 0.05 # Margin above object during "move to a location"
    gripperOriHome=[0, 1, 0, 0]
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    vision_pos[2] = 0.17786528

    #############
    ## Placing ##
    #############

    #if primitive called without placing arguments, go to center of bin
    if rel_pose==None:
        drop_pose = ik.helper.get_params_yaml('bin'+str(binId)+'_pose')
        drop_pose[3:7] = gripperOriHome
    else:
        drop_pose = ik.helper.drop_pose_transform(binId, rel_pose, BoxBody, place_pose, viz_pub, listener, br)

    #~ Adjust height according to weigth
    if is_drop:
        drop_offset = 0.15
    else:
        drop_offset = 0.025
    #~set drop pose target z position to be bottom of bin
    drop_pose[2] = bin_pose[2] - rospy.get_param('tote/height') + drop_offset

    #~Predrop: go to top middle bin surface fast without guarded move
    predrop_pos=np.array(drop_pose[0:3])
    predrop_pos[2] = bin_pose[2] + 0.05

    ######################
    ##  Generate plans ##
    ######################
    #~move safe to placing bin number
    rospy.logdebug('[Picking] Collision free placing to binId %s and position %s', binId, predrop_pos)
#        if isExecute:
#            q_initial = collision_free_placing(binId, listener, obj_ID=objId, BoxBody=BoxBody, isSuction = False,with_object = True, hand_orientation=drop_pose[3:7],projected_target_position = predrop_pos, isReturn = True)
#            if update_command is not None:
#                update_command.execute()

    #~0.go up to avoid collision with tote walls
    current_pose = ik.helper.get_tcp_pose(listener, tcp_offset = l3)
    current_pose[2] = current_pose[2] + 0.15
    plan, qf, plan_possible = generatePlan(q_initial, current_pose[0:3], current_pose[3:7], tip_hand_transform, 'faster',  plan_name = 'go_up')
    if plan_possible:
        plans.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~0.1.fast motion to bin surface high
    predrop_pos_high = predrop_pos
    predrop_pos_high[2] = current_pose[2]
    plan, qf, plan_possible = generatePlan(q_initial, predrop_pos_high, drop_pose[3:7], tip_hand_transform, 'faster',  plan_name = 'go_bin_surface_high')
    if plan_possible:
        plans.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~1.fast motion to bin surface
    predrop_pos[2] = bin_pose[2] + 0.05
    plan, qf, plan_possible = generatePlan(q_initial, predrop_pos, drop_pose[3:7], tip_hand_transform, 'faster',  plan_name = 'go_bin_surface')
    if plan_possible:
        plans.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~2. Guarded move down slow
    plan, qf, plan_possible = generatePlan(q_initial, drop_pose[0:3], drop_pose[3:7], tip_hand_transform, 'fast',   guard_on=WeightGuard(binId, threshold = 100), backwards_speed = 'superSaiyan', plan_name = 'guarded_drop')
    if plan_possible:
        plans_placing.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~3. open gripper
    grasp_plan = EvalPlan('helper.moveGripper(0.110, 200)')
    plans_placing2.append(grasp_plan)

    #~4. open spatula
    grasp_plan = EvalPlan('spatula.open()')
    plans_placing2.append(grasp_plan)

    #~5. go to predrop_pos
    plan, qf, plan_possible = generatePlan(q_initial, predrop_pos[0:3], drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'predrop_pos')
    if plan_possible:
        plans_placing2.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~6. go to final pose
    final_pos = bin_pose[0:3]
    final_pos[2] = bin_pose[2] + 0.3
    plan, qf, plan_possible = generatePlan(q_initial, final_pos, drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'final_pose')
    if plan_possible:
        plans_placing2.append(plan)
        q_initial = qf
    else:
        return compose_output()

    #~ Execute plans
    if plan_possible:
        if isExecute:
            executePlanForward(plans, withPause)

            #We start recording now
            if recorder is not None:
                lasers.start(binId)
                recorder.start_recording(action='placing', action_id=str(datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")), tote_num=binId, frame_rate_ratio=5, image_size=-1)

            rospy.set_param('is_record', True)
            executePlanForward(plans_placing, withPause)
            if rospy.get_param('is_contact'):
                ik.helper.move_cart(dz=0.015)
            executePlanForward(plans_placing2, withPause)
            rospy.set_param('is_record', False)
            if recorder is not None:
                recorder.stop_recording(save_action=True)
                lasers.stop(binId)

        execution_possible = True
        return compose_output()

def unit_test(listener, br):
    # Initialize
    isExecute = True
    rospy.sleep(0.5)
    viz_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
#    data = posesrv('','')

    #~define objId
    TARGET = 'expo_eraser'

    #~define obj poses
    objInput = []
    objInput.append(ik.helper.get_params_yaml('bin0_pose'))
    objInput.append(ik.helper.get_params_yaml('bin1_pose'))
    objInput.append(ik.helper.get_params_yaml('bin2_pose'))

    objInput[0][3:7] = np.array([0.707,0,0.707,0])
    #~1. grasp object
    grasp_dict = grasp(objInput[0],
                  listener,
                  br,
                  isExecute = True,
                  objId = 'expo_eraser',
                  binId=0,
                  withPause = False,
                  viz_pub = None,
                  recorder = None)
    #~2. regrasp object
    regrasp_dict = grasp_correction(objInput[0],
                            np.array([0.02, 0.01, 0.]),
                            listener,
                            br)
    #~3. retrieve object
    regrasp_dict =  retrieve(listener,
                      br,
                      isExecute = True,
                      objId = 'expo_eraser',
                      binId=0,
                      withPause = False,
                      viz_pub = None,
                      recorder = None)
    # #~5. place object
    place_dict = place(listener,
                  br,
                  isExecute = True,
                  binId=0,
                  withPause = False,
                  rel_pose = None,
                  BoxBody = None,
                  place_pose = None,
                  viz_pub = None,
                  is_drop = True,
                  recorder = None)

    #~1. grasp object
    # output_dict = grasp(objInput,
    #               listener,
    #               br,
    #               isExecute = True,
    #               objId = 'expo_eraser',
    #               binId=0,
    #               withPause = False,
    #               viz_pub = None,
    #               recorder = None)

def unit_test_collision(listener, br):
    tcp_pose = np.array([1.0,-.4,0,1,0,0,0])
    delta_pos = np.array([0,0,0])
    check_collision(tcp_pose, delta_pos, listener, br)

# To test the function
if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)

    unit_test(listener=listener, br=br)
    # unit_test_collision(listener=listener, br=br)
