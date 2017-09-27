#!/usr/bin/env python

import rospy
import numpy as np
from numpy import linalg as la
from pr_msgs.msg import *
from manual_fit.srv import *
from copy import deepcopy
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
import tf
import tf.transformations as tfm
from collision_free_placing import collision_free_plan, collision_free_placing, go_arc_safe, is_in_bin
from ik.ik import generatePlan, EvalPlan, WeightGuard, GraspingGuard, executePlanForward, fastik
from ik.helper import get_joints, get_tcp_pose, mat2quat, get_params_yaml, reference_frames, get_object_properties, vision_transform_precise_placing, vision_transform_precise_placing_with_visualization, pose_transform_precise_placing
from ik.roshelper import pose2list, poseTransform
import goToHome
import os
from collision_detection.collisionHelper import collisionCheck, getBinPoints, getFingerPoints, collisionFree
from visualization_msgs.msg import MarkerArray
import gripper, spatula, scorpion, suction
import time
from ik.helper import deleteMarkers, plotPickPoints, plotBoxCorners
from apc.helper import UpdateCommand

def record_rosbag(topics):
    #~ dir_save_bagfile = '/media/' + os.environ['HOME']+'/gelsight_grasping_data'
    dir_save_bagfile = os.environ['ARCDATA_BASE']+'/gelsight_grasping_data'
    make_sure_dir_exists(dir_save_bagfile)
    name_of_bag=time.strftime("%Y%m%d-%H%M%S")
    rosbag_proc = subprocess.Popen('rosbag record -q -O %s %s' % (name_of_bag, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)
    return name_of_bag

def make_sure_dir_exists(path):
    if not os.path.exists(path):
        os.makedirs(path)

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

def get_hand_frame(obj_pose_orient_norm, obj_dim, listener, br):

    #~Define reference frames
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = reference_frames(listener = listener, br=br)
    #~ Project all axes of object about Z-world axis
    proj_vecZ = np.abs(np.dot(world_Z,obj_pose_orient_norm))
    temp = proj_vecZ.argsort()
    
    #~sort all dimensions
    max_index=temp[2]
    secondmax_index=temp[1]
    min_index=temp[0]
    signed_proj_vecZ = np.dot(world_Z,obj_pose_orient_norm)
    if signed_proj_vecZ[max_index]<0:
        hand_Z=obj_pose_orient_norm[:,max_index]
    else:
        hand_Z=-obj_pose_orient_norm[:,max_index]
        
    # find smaller of the other two object dimensions
    obj_xyplane_dim_array = np.array([obj_dim[secondmax_index],obj_dim[min_index]])
    obj_smaller_xydim_index = np.argmin(np.fabs(obj_xyplane_dim_array))
    
    # Set hand X (finger vec) along smaller dimension vector
    if obj_smaller_xydim_index==0:
        hand_X=obj_pose_orient_norm[:,secondmax_index]
        grasp_width=obj_dim[secondmax_index]
    else:
        hand_X=obj_pose_orient_norm[:,min_index]
        grasp_width=obj_dim[min_index]
        
    #~define coordinate frame vectors
    hand_Y=np.cross(hand_Z, hand_X)

    return hand_X, hand_Y, hand_Z, grasp_width

def get_bbox_vision(vision_input):
    
    #~ Object quaternion and position
    vision_input_array = np.asarray(vision_input)
    objQuat = vision_input_array[0:4]
    objPos = vision_input_array[4:7]
    objPose = np.hstack((objPos, objQuat))
    
    #~ Get objPose, objPos, and obj
    obj_pose_tfm_list = matrix_from_xyzquat(objPose)
    obj_pose_tfm = np.array(obj_pose_tfm_list)
    obj_pose_orient_norm=obj_pose_tfm[0:3,0:3]

    obj_X = obj_pose_orient_norm[:,0]
    obj_Y = obj_pose_orient_norm[:,1]
    obj_Z = obj_pose_orient_norm[:,2]

    #~ Get objDim
    minx = vision_input[7]
    miny = vision_input[8]
    minz = vision_input[9]
    obj_dim = [2*minx, 2*miny, 2*minz]

    return (obj_dim, obj_X, obj_Y, obj_Z, obj_pose_orient_norm, objPose)

def get_hand_frame_vert(obj_pose_orient_norm, obj_dim, listener, br):

    #~Define reference frames
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = reference_frames(listener = listener, br=br)

    #~ Project all axes of object about Z-world axis
    proj_vecZ = np.abs(np.dot(world_Z,obj_pose_orient_norm))
    temp = proj_vecZ.argsort()
    max_index=temp[2]
    secondmax_index=temp[1]
    min_index=temp[0]
    
    #~Find maximum 2 dimensions/vectors projections along World Z
    obj_dim_along_maxprojZ=obj_dim[max_index]
    obj_vec_along_maxprojZ=obj_dim_along_maxprojZ*obj_pose_orient_norm[:,max_index]
    obj_dim_along_secmaxprojZ=obj_dim[secondmax_index]
    obj_vec_along_secmaxprojZ=obj_dim_along_secmaxprojZ*obj_pose_orient_norm[:,secondmax_index]
    
    #~Find projection of max 2 dimensions on the plane
    obj_vec_along_diagprojZ=obj_vec_along_maxprojZ+obj_vec_along_secmaxprojZ
    obj_vec_along_diagprojZ_plane=np.dot(np.identity(3)-[[0,0,0],[0,0,0],[0,0,1]],obj_vec_along_diagprojZ)
    obj_dim_along_diagprojZ_plane=la.norm(obj_vec_along_diagprojZ_plane)
    
    #~Find projection of smallest object dimension along Z on the plane
    obj_dim_along_minprojZ=obj_dim[min_index]
    obj_vec_along_minprojZ= obj_dim_along_minprojZ*obj_pose_orient_norm[:,min_index]
    obj_vec_along_minprojZ_plane=np.dot(np.identity(3)-np.array([[0,0,0],[0,0,0],[0,0,1]]),obj_vec_along_minprojZ)
    obj_dim_along_minprojZ_plane=la.norm(obj_vec_along_minprojZ_plane)
    
    #~Find minimum dimension on the plane
    temp_array = np.array([obj_dim_along_diagprojZ_plane, obj_dim_along_minprojZ_plane])
    obj_min_plane = np.min(np.fabs(temp_array))
    obj_min_plane_index = np.argmin(np.fabs(temp_array))
    
    #~Choose right vector direction
    if obj_min_plane_index==0:
        hand_X = obj_vec_along_diagprojZ_plane/la.norm(obj_vec_along_diagprojZ_plane)
        grasp_width = obj_dim_along_diagprojZ_plane
    else:
        hand_X = obj_vec_along_minprojZ_plane/la.norm(obj_vec_along_minprojZ_plane)
        grasp_width = obj_dim_along_minprojZ_plane
        
    #~Define two remaining frames
    hand_Z=np.array([0,0,-1])
    
    # Set hand Y axis
    hand_Y=np.cross(hand_Z, hand_X)

    return hand_X, hand_Y, hand_Z, grasp_width

def get_picking_params_from_12(objInput):
    
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

def grasp(objInput,
          listener,
          br,
          isExecute = True,
          objId = 'expo_eraser',
          binId=0,
          flag = 0,
          withPause = False,
          rel_pose = None,
          BoxBody = None,
          place_pose = None,
          viz_pub = None,
          is_drop = True,
          update_command = None):

    #########################################################
    # nikhilcd and fhogan
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
    # 1) grasp_possible: Is there an object in the hand? (True/False)
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
    
    rospy.loginfo('[Picking] Starting primitive with flag: %d' % flag)
    
    rospy.loginfo('[Picking] objInput %s', objInput)
    rospy.loginfo('[Picking] objId %s', objId)
    rospy.loginfo('[Picking] flag %s', flag)
    rospy.loginfo('[Picking] withPause %s', withPause)
    rospy.loginfo('[Picking] rel_pose %s', rel_pose)
    rospy.loginfo('[Picking] BoxBody %s', BoxBody)
    rospy.loginfo('[Picking] place_pose %s', place_pose)

    #~Initialize values
    #~ tcp_initial = get_tcp_pose(listener)
    q_initial = get_joints()
    bin_pose = get_params_yaml('bin'+str(binId)+'_pose')
    bin3_pose = get_params_yaml('bin3_pose')
    grasp_limit=rospy.get_param("/gripper/opening_limit_in")
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    joint_topic = '/joint_states'
    plans = []
    grasp_possible = False
    graspPose=[]
    plan_possible = False
    execution_possible = False
    gelsight_data=[]
    gripper_opening = 0.0
    collision = False
    final_object_pose=None
    def compose_output():
        return {'collision':collision,'grasp_possible':grasp_possible,'plan_possible':plan_possible,'execution_possible':execution_possible,'gripper_opening':gripper_opening,'graspPose':graspPose,'gelsight_data':gelsight_data,'final_object_pose':final_object_pose}

    # Specify gelsight topics to record
    gelsight_topics = ["/rpi/gelsight/raw_image","/rpi/gelsight/deflection"]

    #~Constants parameters
    pregrasp_width_scaling = 1.6
    UpdistFromBin = 0.1 # Margin above object during "move to a location" 
    UpdistFromObject = 0.05 # Margin above object during "move to a location" 
    gripperOriHome=[0, 1, 0, 0]
    gripperOriVision=[0.7071, -0.7071, 0, 0]
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    link_6_hand_transform = [0., 0., 0., 0,0,0]

    #~Initialize lists
    homePosition    = []
    homeOrientation = []
    dropPositon     = []
    dropOrientation = []
    
    #~active vision and drop motions
    ###################
    ## Active vision ##
    ###################
    #~define active vision home pose
    delta_vision_pose = get_params_yaml('vision_pose_picking')
    vision_pos=np.array(bin_pose[0:3])+np.array(delta_vision_pose[0:3])
    
    if binId == 3:
        vision_pos[2] = 0.23087622
    else:
        vision_pos[2] = 0.17786528
    
    if flag == 1:
        rospy.loginfo('[Picking] Enter flag 1')
        #~execute collision free plan
        if isExecute:#~ and not is_in_bin(listener, bin_id = binId):
            q_initial = collision_free_placing(binId, listener, obj_ID=objId, BoxBody=None, isSuction = False,with_object = True,hand_orientation=delta_vision_pose[3:7],projected_target_position = vision_pos[0:3], isReturn = True)
            
        plan, qf, plan_possible = generatePlan(q_initial, vision_pos[0:3], delta_vision_pose[3:7], tip_hand_transform, 'faster', plan_name = 'go_active_vision')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
            
        if isExecute and plan_possible:
            executePlanForward(plans, withPause)
        execution_possible = True
        return compose_output()
        
    #################
    ## Drop object ##
    #################
    elif flag == 2:
        #if primitive called without placing arguments, go to center of bin
        if rel_pose==None:
            drop_pose = get_params_yaml('bin'+str(binId)+'_pose')
            drop_pose[3:7] = gripperOriHome
        else:
            #~define gripper home orientation
            base_pose = [0.,0.,0.,0.,1.,0.,0.] #~goarc hand pose
            matrix_base_pose= tfm.quaternion_matrix(base_pose[3:7])
            hand_orient_norm = matrix_base_pose[0:3,0:3]
            #~initialize arguments for placing functions
            finger_opening = gripper.getGripperopening()
            safety_margin=.035
            #~ get 3d bin and finger points
            finger_pts_3d = getFingerPoints(finger_opening, [0,0,0], hand_orient_norm, False)
            bin_pts_3d = getBinPoints(binId=binId,listener=listener, br=br)
            #~convert to 2d
            bin_pts = bin_pts_3d[:,0:2]
            finger_pts = finger_pts_3d[:,0:2]
            #~ ~perform coordinate transformation from object to gripper 
            (drop_pose,final_object_pose) = pose_transform_precise_placing(rel_pose, BoxBody, place_pose, base_pose, bin_pts_3d, finger_pts, safety_margin, False, viz_pub)
        #~frank hack for safety in z direction
        if is_drop:
            drop_offset = 0.15
        else:
            drop_offset = 0.002
            
        drop_pose[2] = bin_pose[2] - rospy.get_param('bin'+str(binId)+'/height') + drop_offset
        ###################
        ## Target points ##
        ###################
        
        #~define predrop position
        predrop_pos=np.array(drop_pose[0:3])
        predrop_pos[2]=bin_pose[2] + 0.1 
        
        ######################
        ##  Generate plans ##
        ######################
        #~move safe to placing section
        rospy.logdebug('[Picking] Collision free placing to binId %s and position %s', binId, predrop_pos)
        if isExecute:
            q_initial = collision_free_placing(binId, listener, obj_ID=objId, BoxBody=BoxBody, isSuction = False,with_object = True, hand_orientation=drop_pose[3:7],projected_target_position = predrop_pos, isReturn = True)
            if update_command is not None:
                update_command.execute()
        map_bin_id_to_ws_id = {0:0,1:1,2:2,3:3,4:0,5:0,6:4,7:4,8:5}
        #~fast motion to bin surface
        fast_drop_pos = drop_pose[0:3]
        fast_drop_pos[2] = bin_pose[2] + 0.1
        plan, qf, plan_possible = generatePlan(q_initial, fast_drop_pos, drop_pose[3:7], tip_hand_transform, 'faster',  plan_name = 'go_bin_surface')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~guarded move down slow
        plan, qf, plan_possible = generatePlan(q_initial, drop_pose[0:3], drop_pose[3:7], tip_hand_transform, 'fast',   guard_on=WeightGuard(map_bin_id_to_ws_id[binId]), backwards_speed = 'superSaiyan', plan_name = 'guarded_drop')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
        #~open gripper
        grasp_plan = EvalPlan('moveGripper(0.110, 200)')
        plans.append(grasp_plan)
        #~open spatula
        grasp_plan = EvalPlan('spatula.open()')
        plans.append(grasp_plan)
        #~go to predrop_pos
        plan, qf, plan_possible = generatePlan(q_initial, fast_drop_pos[0:3], drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'predrop_pos')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
        #~go to final pose
        final_pos = bin_pose[0:3]
        final_pos[2] = bin_pose[2] + 0.3
        plan, qf, plan_possible = generatePlan(q_initial, final_pos, drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'final_pose')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()

        if plan_possible:
            if isExecute:
                executePlanForward(plans, withPause)
            execution_possible = True
            return compose_output()
            
    #######################
    ## Picking primitive ##
    #######################
    #~Define reference frames
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = reference_frames(listener= listener, br=br)
    
    #~Get object parameters
    if len(objInput)==7:
        try:
            obj_dim, obj_X, obj_Y, obj_Z, obj_pose_orient_norm = get_object_properties(objId,objInput)
        except:
            obj_dim = np.array([0.135, 0.055, 0.037])
            obj_X = np.array([0.0, 0.0, 1.0])
            obj_Y = np.array([0.0, 1.0, 0.0])
            obj_Z = np.array([-1.0, 0.0, 0.0])
            obj_pose_orient_norm = np.array([[ 0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])

        objPose = deepcopy(objInput)
        graspPos = objPose[0:3]
        hand_X, hand_Y, hand_Z, grasp_width = get_hand_frame(obj_pose_orient_norm=obj_pose_orient_norm, obj_dim=obj_dim, listener=listener, br=br)

    elif len(objInput)==12:
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = get_picking_params_from_12(objInput)

    elif len(objInput)==14:
        obj_dim, obj_X, obj_Y, obj_Z, obj_pose_orient_norm, objPose = get_bbox_vision(objInput)
        graspPos = objPose[0:3]
        #~Define hand frame
        hand_X, hand_Y, hand_Z, grasp_width = get_hand_frame(obj_pose_orient_norm=obj_pose_orient_norm, obj_dim=obj_dim, listener=listener, br=br)
    else:
        rospy.logwarn('[Picking] objInput size is not correct')
        
    
    #~Hack: prevent collision with post
    #~ proj_handZ_plane = np.dot(hand_Z, world_X)
    #~ if proj_handZ_plane < 0:
        #~ hand_X, hand_Y, hand_Z, grasp_width = get_hand_frame_vert(obj_pose_orient_norm=obj_pose_orient_norm, obj_dim=obj_dim, listener=listener, br=br)

    hand_orient_norm = np.vstack([hand_X,hand_Y,hand_Z])
    hand_orient_norm=hand_orient_norm.transpose()

    #~Define gripper opening
    grasp_width_pregrasp=pregrasp_width_scaling*grasp_width

    #~Check if object fits in gripper opening
    if grasp_width < grasp_limit:
        grasp_possible = True
    if grasp_width_pregrasp >= grasp_limit:
        grasp_width_pregrasp = grasp_limit

    #~Run collision detection
    tcpPos = graspPos - hand_Z*spatula_tip_to_tcp_dist

    #~ collision = collisionCheck(grasp_width_pregrasp, tcpPos, hand_orient_norm, False, False)
    #~ collision = collisionCheck(finger_opening= grasp_width_pregrasp,
                               #~ tcp_pos=tcpPos,
                               #~ hand_orient_norm=hand_orient_norm,
                               #~ listener=listener,
                               #~ br=br,
                               #~ isSuction=False,
                               #~ flagPlt=False)
    #~Grasp relocation script
    
    
    hand_orient_quat=mat2quat(hand_orient_norm)
    euler = tf.transformations.euler_from_quaternion(hand_orient_quat)
    theta = euler[2] - np.pi
    #~ sys.exit()
    colFreePose = collisionFree(graspPos, binId=binId, listener=listener, br=br, finger_opening = grasp_width_pregrasp, safety_margin = 0.00, theta = theta)
    graspPos[0:2] = colFreePose[0:2]
    #~ collision = Fal
    rospy.logdebug('[Picking] collision %s',collision)

    # graspPose is computed just for data logging. No practical use.
    graspPose=np.hstack((graspPos,hand_orient_quat))

    if collision:
        execution_possible = False
        rospy.logwarn('[Picking] collision %s',collision)

    #################################
    ## GENERATE PLANS OF PRIMITIVE ##
    #################################.
    #~ Start from home position
    if isExecute:
        q_initial = collision_free_placing(binId, listener, isSuction = False, with_object = False, hand_orientation=[0,1,0,0], projected_target_position = bin_pose[0:3], isReturn = True)
        scorpion.back()
        
    if  grasp_possible and not collision:
        #~Move to a location above the bin 
        #~ Rotate gripper to grasp orientation
        pregrasp_targetPosition = graspPos - hand_Z*UpdistFromObject
        pregrasp_targetPosition[2] = bin_pose[2]
        plan, qf, plan_possible = generatePlan(q_initial, pregrasp_targetPosition, hand_orient_quat, tip_hand_transform, 'superSaiyan', plan_name = 'rotate_gripper')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
       #~ Open gripper
        grasp_plan = EvalPlan('moveGripper(%f, 200)' % grasp_width_pregrasp)
        plans.append(grasp_plan)

        #~Open spatula  
        grasp_plan = EvalPlan('spatula.open()')
        plans.append(grasp_plan)
        
       #~Go down  
        grasp_targetPosition=deepcopy(graspPos)
        rospy.loginfo('[Picking] Grasp Target %s', grasp_targetPosition)
        
        #~sleep
        sleep_plan = EvalPlan('rospy.sleep(0.2)')
        plans.append(sleep_plan)
        
        #~################################
        #~frank hack for safety in z direction
        grasp_targetPosition[2] = bin_pose[2] -  rospy.get_param('/bin'+str(binId)+'/height') + 0.000 #~frank hack for safety
        if binId==0:
            grasp_targetPosition[2] = grasp_targetPosition[2] +0.005
        #~################################
        plan, qf, plan_possible = generatePlan(q_initial, grasp_targetPosition, hand_orient_quat, tip_hand_transform, 'faster', guard_on=WeightGuard(binId, threshold = 100), plan_name = 'guarded_pick')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
        #~Close spatula
        grasp_plan = EvalPlan('spatula.close()')
        plans.append(grasp_plan)
        
        #~Close gripper
        grasp_plan = EvalPlan('graspinGripper(%f,%f)'%(200,50))
        plans.append(grasp_plan)
        
        #~sleep
        sleep_plan = EvalPlan('rospy.sleep(0.2)')
        plans.append(sleep_plan)

        #~Move to a location above the bin
        plan, qf, plan_possible = generatePlan(q_initial, vision_pos, delta_vision_pose[3:7], tip_hand_transform, 'fastest', plan_name = 'go_to_active_vision')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()

    ################
    ## EXECUTION ###
    ################
    if isExecute and plan_possible:
        executePlanForward(plans, withPause)

    #~Check if picking success
    low_threshold = 0.0035
    high_threshold = 0.006
    if isExecute and plan_possible and grasp_possible and not collision:
        drop_thick=0.005
        rospy.sleep(0.2)
        gripper_opening=gripper.getGripperopening()
        if gripper_opening > high_threshold:
            rospy.loginfo('[Picking] ***************')
            rospy.loginfo('[Picking] Pick Successful (Gripper Opening Test)')
            rospy.loginfo('[Picking] ***************')
            execution_possible = True
        else:
            rospy.loginfo('[Picking] ***************')
            rospy.loginfo( '[Picking] Pick Inconclusive (Gripper Opening Test)')
            rospy.loginfo( '[Picking] ***************')
            execution_possible = None
            
    elif not isExecute and not collision:
        execution_possible=plan_possible

    #~ deleteMarkers(viz_pub,'object_bounding_box_for_collision')
    #~ deleteMarkers(viz_pub,'object_bounding_box_for_collision2')
    return compose_output()

def unit_test(listener, br):
    # Initialize
    isExecute = True
    #~ goToHome.prepGripperPicking()
#    posesrv = rospy.ServiceProxy('/manualfit_object_pose', GetPose)   # should move service name outsoft_white_lightbulb
    rospy.sleep(0.5)
#    data = posesrv('','')
    viz_pub=rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=100)
    #~ goToHome.prepGripperPicking()

    #~define objId 
    TARGET = 'expo_eraser'
    
    #~define obj poses 
    objInput = []
    objInput.append(get_params_yaml('bin0_pose'))
    objInput.append(get_params_yaml('bin1_pose'))
    objInput.append(get_params_yaml('bin2_pose'))
#    objInput.append([1.01179456711, -0.196397691965, -0.261632657051, 0.0, -0.707106769085, 0.0, 0.707106769085])
#    objInput.append([1.01179456711, 0.190555930138, -0.261632627249, 0.0, -0.707106769085, 0.0, 0.707106769085])
#    objInput.append([1.01179456711, 0.560784399509, -0.261632627249, 0.0, -0.707106769085, 0.0, 0.707106769085])
#    
    #~unit test for flags 1 and 2 in bins 0 and 1 (with weight guard)
    flag_list = [0,1,2]
    bin_list = [0,1,2]
    #~ bin_list = [0]
    for binId in bin_list:
        for flag in flag_list:
            if flag==2:
                bin_list_drop = [binId] 
                for bin_drop_id in  bin_list_drop:
                    (output_dict)=grasp(objInput=objInput[binId],
                                        listener=listener,
                                        br=br,
                                        isExecute=isExecute,
                                        objId=TARGET,
                                        binId=bin_drop_id,
                                        flag=flag,
                                        withPause = False)
                    gripper.close()
            else:
                #~call pick function without xtra placing arguments (flag:0,1)
                (output_dict)=grasp(objInput[binId],
                                    listener=listener,
                                    br=br,
                                    isExecute=isExecute,
                                    objId=TARGET,
                                    binId=binId,
                                    flag=flag,
                                    withPause = False)
                                    
def unit_test_collision(listener, br):
    # Initialize
    isExecute = True
    goToHome.prepGripperPicking()
    posesrv = rospy.ServiceProxy('/manualfit_object_pose', GetPose)   # should move service name outsoft_white_lightbulb
    rospy.sleep(0.5)
    data = posesrv('','')
    viz_pub=rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=100)
    #~ goToHome.prepGripperPicking()

    #~define objId 
    TARGET = 'crayons'
    
    #~define obj poses 
    objInput = []
    objInput.append([0.889025449753, -0.854251086712, -0.12657110393, 0.232909590006, -0.66764742136, 0.232909649611, 0.66764742136])
    #~bin0
    objInput.append([1.01179456711, 1.34244477749, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([0.459975481033, -0.583415150642, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.01179456711, -1.33003950119, 0.0363663174212, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.28368413448, -0.542792618275, -0.126571059227, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~bin1
    objInput.append([1.01179456711, 1.34244477749, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([0.459975481033, -0.137250989676, -0.126571148634, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.01179456711, -1.33003950119, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.28368413448, -0.176122009754, -0.126571029425, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~bin2
    objInput.append([1.01179456711, 1.34244477749, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([0.459975481033, 0.201511979103, -0.126571118832, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.01179456711, -1.33003950119, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.28368413448, 0.166815787554, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~bin3
    objInput.append([1.01179456711, 1.34244477749, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([0.459975481033, 0.654191792011, -0.126571089029, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.01179456711, -1.33003950119, -0.126570999622, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.28368413448, 0.625417530537, -0.12657096982, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~unit test for flags 1 and 2 in bins 0 and 1 (with weight guard)
    bin_list = [0,1,2,3]
    #~ bin_list = [0]
    flag = 0
    #~first angle case
    counter=0
    output_dict=grasp(objInput=objInput[counter],
                                listener=listener,
                                br=br,
                                isExecute=isExecute,
                                objId=TARGET,
                                binId=0,
                                flag=flag,
                                withPause = False)
    counter+=1
    #~rest
    for binId in bin_list:
        for wall in range(4):
            (output_dict)=grasp(objInput=objInput[counter],
                                listener=listener,
                                br=br,
                                isExecute=isExecute,
                                objId=TARGET,
                                binId=binId,
                                flag=flag,
                                withPause = False)
            counter+=1


def rviz_test(binId, flag, listener, br):
    #~initialize
    isExecute = True
    posesrv = rospy.ServiceProxy('/manualfit_object_pose', GetPose)   # should move service name outsoft_white_lightbulb
    rospy.sleep(0.5)
    data = posesrv('','')
    #~ goToHome.prepGripperPicking()

    #~define objId 
    TARGET = 'expo_eraser'
    
    #~call pick function
    objInput = pose2list(data.pa.object_list[0].pose)
    (output_dict)=grasp(objInput=objInput,
                        listener=listener,
                        br=br,
                        isExecute=isExecute,
                        objId=TARGET,
                        binId=binId,
                        flag=flag,
                        withPause = True)
                    
def placing_test(binId, flag, listener, br):
    isExecute = True
    posesrv = rospy.ServiceProxy('/manualfit_object_pose', GetPose)   # should move service name outsoft_white_lightbulb
    rospy.sleep(0.5)
    data = posesrv('','')
    TARGET = 'expo_eraser'

    box_rot=tfm.euler_matrix(0, 0, 0.0*np.pi/(180.0), axes='sxyz')
    box_rot[:,2]=-box_rot[:,2]
    box_rot[:,1]=-box_rot[:,1]
    box_quat=tfm.quaternion_from_matrix(box_rot).tolist()
    box_pos=[.0,0.0,0.0]
    box_pose=box_pos+box_quat

    place_rot=tfm.euler_matrix(0, 0, 30.0*np.pi/(180.0), axes='sxyz')
    place_quat=tfm.quaternion_from_matrix(place_rot).tolist()
    place_pos=[1.,-0.224820479751,0.0]
    place_pose=place_pos+place_quat
    
    map_box_pose = poseTransform(box_pose, "link_6", "map", listener)
    box_dim=[.07,.03,.2]
    bbox_info=map_box_pose[3:7]+map_box_pose[0:3]+box_dim
    
    viz_pub=rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    rospy.sleep(0.5)
    vision_transform_precise_placing_with_visualization(bbox_info,viz_pub, listener=listener)
    rospy.sleep(0.5)

    (rel_pose,BoxBody)=vision_transform_precise_placing(bbox_info=bbox_info, listener=listener)

    objInput = []
    
    (output_dict) = grasp(objInput=objInput,
                          listener=listener,
                          br=br,
                          isExecute=isExecute,
                          objId=TARGET,
                          binId=binId,
                          flag=flag,
                          withPause = True,
                          rel_pose=rel_pose,
                          BoxBody=BoxBody,
                          place_pose=place_pose)

# To test the function
if __name__=='__main__':
    #~ Frank & Nikhil:hack to get proper vision format
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)

    
    unit_test(listener=listener, br=br)
    #~ unit_test_collision(listener=listener, br=br)
    #~ rviz_test(binId=0, flag=0, listener=listener, br=br)
    #~ placing_test(binId=8, flag=2, listener=listener, br=br)
    #~ collision_free_placing(6, listener=listener, isSuction=False)

    

