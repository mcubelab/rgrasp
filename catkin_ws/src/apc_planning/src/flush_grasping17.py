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
from collision_free_placing import collision_free_plan, collision_free_placing, go_arc_safe
from ik.ik import generatePlan, EvalPlan, WeightGuard, GraspingGuard, executePlanForward, fastik
from ik.helper import get_joints, get_tcp_pose, mat2quat, get_params_yaml, reference_frames, get_object_properties, vision_transform_precise_placing, vision_transform_precise_placing_with_visualization, pose_transform_precise_placing
from ik.roshelper import pose2list, poseTransform
import goToHome
import os
from collision_detection.collisionHelper import collisionCheck, getBinPoints, getFingerPoints
from visualization_msgs.msg import MarkerArray
import gripper, spatula, scorpion, suction
import time
from ik.helper import deleteMarkers, plotPickPoints
from apc.helper import UpdateCommand
from ik.visualize_helper import visualize_flush_proposals

havegripper = rospy.get_param('/have_gripper', True)

def record_rosbag(topics):
    #dir_save_bagfile = os.environ['HOME']+'/gelsight_flushgrasping_data'
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

def get_flush_hand_frame(objInput, binId, listener, br):
    #~Define reference frames
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = reference_frames(listener=listener, br=br)
    bin_center_pose = get_params_yaml('bin'+str(binId)+'_pose')
    bin_center_pos=np.array(bin_center_pose[0:3])
    bin_width=rospy.get_param("/tote/width")
    gelsight_fing_offset=0.050

    if  objInput[1]>bin_center_pos[1]:
        flush_mode='left'

        hand_Z=np.array(-world_Z)
        hand_X=np.array(world_Y)
        hand_Y=np.array(world_X)

        grasp_posY = bin_center_pos[1]+bin_width/2.0-objInput[4]/2.0-gelsight_fing_offset
        grasp_posX = objInput[0]
        grasp_posZ = objInput[2]-objInput[3]
        graspPos = np.array([grasp_posX, grasp_posY, grasp_posZ])

    else:
        flush_mode='right'

        hand_Z=np.array(-world_Z)
        hand_X=np.array(-world_Y)
        hand_Y=np.array(-world_X)
        if binId==0:
            gelsight_fing_offset = 0.04

        grasp_posY = bin_center_pos[1]-bin_width/2.0+objInput[4]/2.0+gelsight_fing_offset
        grasp_posX = objInput[0]
        grasp_posZ = objInput[2]-objInput[3]
        graspPos = np.array([grasp_posX, grasp_posY, grasp_posZ])

    grasp_width = objInput[4]
    return graspPos, hand_X, hand_Y, hand_Z, grasp_width

def flush_grasp(objInput,
                listener,
                br,
                isExecute = True,
                objId = 'expo_eraser',
                binId=0,
                flag = 0,
                withPause = False,
                rel_pose=None,
                BoxBody=None,
                place_pose=None,
                viz_pub=None,
                is_drop = True,
                update_command = None):

    #########################################################
    # nikhilcd and fhogan
    # Description:
    #~2017 flush picking primitive
    #
    #~Usage ex:
    #~ pick(objInput, listener, isExecute = True, objId = 'cheezit_big_original', flag = 2, withPause = False, drop_pose = get_params_yaml('bin3_pose')):
    #~ pick(objInput, isExecute = True, objId = 'cheezit_big_original', flag = 2, withPause = False, rel_pose=None, BoxBody=None, place_pose=None):
    #
    #~Parameters:
    #
    #~Inputs:
    # 1) objInput: Input from vision or RViz (list of 6 numbers)
    # 2) isExecute: execute the robot motion if possible or not? (Bool)
    # 3) objId: name of object (str)
    # 4) binId: bin in which we need to place the objects. (It is useless when flag 0 and 1)
    # 4) flag: Type of action directed by planning (0:run flush picking, 1:go to home (camera), 2: drop the object at target location)
    # 5) withPause: Pause between robot motions (Bool)
    # 6) rel_pose: object pose relative to link_6 
    # 7) BoxBody: Bounding box properties
    # 8) place_pose: desired position of bounding box

    #~Output:
    # Dictionary of following keys:
    # 1) grasp_possible: Is there an object in the hand? (True/False)
    # 2) plan_possible: Has the plan succeeded? (True/False)
    # 3) execution_possible: Is there anything in the grasp? (True/False)
    # 4) gripper_opening: gripper opening after grasp attempt (in meters)
    # 5) graspPose: pose of the gripper at grasp (7X1 - position and quaternion)
    # 6) gelsight_data: [] , dummy for now
    #########################################################
    
    rospy.loginfo('[Flush Picking] Starting primitive with flag: %d' % flag)
    
    rospy.loginfo('[Flush Picking] objInput %s', objInput)
    rospy.loginfo('[Flush Picking] objId %s', objId)
    rospy.loginfo('[Flush Picking] flag %s', flag)
    rospy.loginfo('[Flush Picking] withPause %s', withPause)
    rospy.loginfo('[Flush Picking] rel_pose %s', rel_pose)
    rospy.loginfo('[Flush Picking] BoxBody %s', BoxBody)
    rospy.loginfo('[Flush Picking] place_pose %s', place_pose)
    
    #~Initialize values
    #~ q_initial = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014] # same q as goToARC
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
    UpdistFromBin=0.1 # Margin above bin during "move to a location" above the bin execution
    gripperOriHome=[0, 1, 0, 0]
    gripperOriVision=[0.7071, -0.7071, 0, 0]
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]

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
        if binId==1:
            drop_pose[2] = drop_pose[2]+0.005
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
        plan, qf, plan_possible = generatePlan(q_initial, predrop_pos[0:3], drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'predrop_pos')
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

    #~Get object parameters
    if len(objInput)==6:
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = get_flush_hand_frame(objInput=objInput,binId=binId,listener=listener, br=br)
        hand_orient_norm = np.vstack([hand_X,hand_Y,hand_Z])
        hand_orient_norm=hand_orient_norm.transpose()

        #~Define gripper opening
        grasp_width_pregrasp=pregrasp_width_scaling*grasp_width

        #~Check if object fits in gripper opening
        if grasp_width < grasp_limit:
            grasp_possible = True
        else:
            rospy.loginfo('[Flush picking] flush Picking not possible!')
        if grasp_width_pregrasp >= grasp_limit:
            grasp_width_pregrasp = grasp_limit
        hand_orient_quat=mat2quat(hand_orient_norm)

        # graspPose is computed just for data logging. No practical use.
        graspPose=np.hstack((graspPos,hand_orient_quat))

    else:
        rospy.loginfo('[Flush picking] objInput size is not correct')
        return compose_output()

    #################################
    ## GENERATE PLANS OF PRIMITIVE ##
    #################################
    #~ Start from home position
    if isExecute:
        q_initial = collision_free_placing(binId, listener, isSuction = False, with_object = False, hand_orientation=[0,1,0,0], projected_target_position = bin_pose[0:3], isReturn = True)
    
    if  grasp_possible:
       #~Move to a location above the bin 
        UPtargetPosition = np.array(bin_pose[0:3])+np.array([0, 0, UpdistFromBin])
        plan, qf, plan_possible = generatePlan(q_initial, UPtargetPosition, gripperOriHome, tip_hand_transform, 'superSaiyan', plan_name = 'go_above_bin')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~ Rotate gripper to grasp orientation 
        pregrasp_targetPosition = graspPos - hand_Z*UpdistFromBin
        pregrasp_targetPosition[2] = UPtargetPosition[2]
        plan, qf, plan_possible = generatePlan(q_initial, pregrasp_targetPosition, hand_orient_quat, tip_hand_transform, 'superSaiyan', plan_name = 'rotate_gripper')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
       #~ Close gripper
        grasp_plan = EvalPlan('moveGripper(0.001, 200)')
        plans.append(grasp_plan)
        
        #~ Go Inside the bin
        pregrasp_targetPosition2 = deepcopy(pregrasp_targetPosition)
        bin_height=rospy.get_param("/tote/height")
        pregrasp_targetPosition2[2]=bin_pose[2]- 0.033 # Go 33 mm inside the tote

        plan, qf, plan_possible = generatePlan(q_initial, pregrasp_targetPosition2, hand_orient_quat, tip_hand_transform, 'faster', guard_on=WeightGuard(binId), plan_name = 'guarded_descent_to_wall')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~ Open spatula
        grasp_plan = EvalPlan('spatula.open()')
        plans.append(grasp_plan)

        #~ Open gripper with force control
        grasp_plan = EvalPlan('graspoutGripper(%f,%f)' % (200,12))
        plans.append(grasp_plan)

        if havegripper:
            sleep_plan=EvalPlan('rospy.sleep(0.5)')
            plans.append(sleep_plan)
        
        #~ Go down
        grasp_targetPosition=deepcopy(graspPos)
        #~################################
        #~frank hack for safety in z direction
        grasp_targetPosition[2] = bin_pose[2] -  rospy.get_param('/tote/height') + 0.00 #~frank hack for safety
        if binId==1:
            grasp_targetPosition[2] = grasp_targetPosition[2]+0.005
        #~################################
        plan, qf, plan_possible = generatePlan(q_initial, grasp_targetPosition, hand_orient_quat, tip_hand_transform, 'faster', guard_on=GraspingGuard(), plan_name = 'gelsigh_guarded_descent')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
        #~Close spatula
        grasp_plan = EvalPlan('spatula.close()')
        plans.append(grasp_plan)
        #~Close gripper
        grasp_plan=EvalPlan('moveGripper(roshelper.getGripperopening()-%f,%f)'%(0.005,200))
        plans.append(grasp_plan)
        #~sleep
        if havegripper:
            sleep_temp=EvalPlan('rospy.sleep(0.5)')
            plans.append(sleep_temp)

        grasp_plan = EvalPlan('graspinGripper(%f,%f)'%(50,200))
        plans.append(grasp_plan)
        #sleep
        if havegripper:
            sleep_plan = EvalPlan('rospy.sleep(0.5)')
            plans.append(sleep_plan)

        #~Move to a location above the bin
        plan, qf, plan_possible = generatePlan(q_initial, UPtargetPosition, hand_orient_quat, tip_hand_transform, 'fastest', plan_name = 'go_above_bin')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()

        #~Move to a (collision free safe) position above the bin
        plan, qf, plan_possible = generatePlan(q_initial, vision_pos, delta_vision_pose[3:7], tip_hand_transform, 'fastest', plan_name = 'go_active_vision')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()

    ################
    ## EXECUTION ###
    ################
    #~ Start from home position
    if isExecute and plan_possible:
        #~ goToARC()
        #~ if rospy.get_param('have_robot'):
            #~ gelsight_data=record_rosbag(gelsight_topics)

        # if havegripper:
        #     rospy.sleep(1)
        executePlanForward(plans, withPause)
        #~ terminate_ros_node("/record")
        # if havegripper:
        #     rospy.sleep(1)
    
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
    
def unit_test():
    isExecute = True
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.3)
    posesrv = rospy.ServiceProxy('/manualfit_object_pose', GetPose)   # should move service name outsoft_white_lightbulb
    rospy.sleep(0.5)
    data = posesrv('','')
    TARGET = 'expo_eraser'
    goToHome.prepGripperPicking()
    
    objInput = []
    objInput.append([ 1.01179456711, -0.415803313255, -0.135151088238, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([0.731192350388, -0.575393080711, -0.13515111804, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.00677216053, -0.729377031326, -0.135151103139, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([1.26140773296, -0.580585181713, -0.135151058435, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([ 1.01179456711, -0.415803313255+.3634, -0.135151088238, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([0.731192350388, -0.575393080711+.3634, -0.13515111804, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.00677216053, -0.729377031326+.3634, -0.135151103139, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([1.26140773296, -0.580585181713+.3634, -0.135151058435, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([ 1.01179456711, -0.415803313255+.3634+.3634, -0.135151088238, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([0.731192350388, -0.575393080711, -0.13515111804, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.00677216053, -0.729377031326+.3634+.3634, -0.135151103139, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([1.26140773296, -0.580585181713, -0.135151058435, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([ 1.01179456711, -0.415803313255+.3634+.3634+.3634, -0.135151088238, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([0.731192350388, -0.575393080711+.3634, -0.13515111804, 0.0, -0.707106769085, 0.0, 0.707106769085])
    objInput.append([1.00677216053, -0.729377031326+.3634+.3634+.3634, -0.135151103139, 0.0, -0.707106769085, 0.0, 0.707106769085])
    #~ objInput.append([1.26140773296, -0.580585181713+.3634, -0.135151058435, 0.0, -0.707106769085, 0.0, 0.707106769085])

    #~unit test for flags 1 and 2 in bins 0 and 1 (with weight guard)
    flag_list = [0,1,2]
    bin_list = [0,1]
    wall_list = [0,1]
    counter = 0
    flag = 0
    for binId in bin_list:
        for flag in flag_list:
            if flag ==0:
                for wall in wall_list:
                    objInput_temp = objInput[counter]
                    objInputUnit =objInput_temp[0:3]+[0.050, 0.080, 0.7]
                    (output_dict)=flush_grasp(objInput=objInputUnit,
                                              listener=listener,
                                              br=br,
                                              isExecute=isExecute,
                                              objId=TARGET,
                                              binId=binId,
                                              flag=flag,
                                              withPause = False)
                    counter += 1
            elif flag==1:
                objInput_temp = objInput[counter-1]
                objInputUnit =objInput_temp[0:3]+[0.050, 0.080, 0.7]
                (output_dict)=flush_grasp(objInput=objInputUnit,
                                          listener=listener,
                                          br=br,
                                          isExecute=isExecute,
                                          objId=TARGET,
                                          binId=binId,
                                          flag=flag,
                                          withPause = False)
            elif flag==2:
                objInput_temp = objInput[counter-1]
                objInputUnit =objInput_temp[0:3]+[0.050, 0.080, 0.7]
                
                box_rot=tfm.euler_matrix(0, 0, 180.0*np.pi/(180.0), axes='sxyz')
                box_rot[:,2]=-box_rot[:,2]
                box_rot[:,1]=-box_rot[:,1]
                box_quat=tfm.quaternion_from_matrix(box_rot).tolist()
                box_pos=[0,0.0,0.0]
                box_pose=box_pos+box_quat

                place_rot=tfm.euler_matrix(0, 0, -0.0*np.pi/(180.0), axes='sxyz')
                place_quat=tfm.quaternion_from_matrix(place_rot).tolist()
                place_pos_tmp = get_params_yaml('bin'+str(binId)+'_pose')
                place_pos=place_pos_tmp[0:3]
                place_pose=place_pos+place_quat
                map_box_pose = poseTransform(box_pose, "link_6", "map", listener)
                box_dim=[.07,.03,.2]
                bbox_info=map_box_pose[3:7]+map_box_pose[0:3]+box_dim

                #~ viz_pub=rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
                (rel_pose,BoxBody)=vision_transform_precise_placing(bbox_info, listener)
                #~ rospy.sleep(0.5)

                (rel_pose,BoxBody)=vision_transform_precise_placing(bbox_info, listener)
    
    
                (output_dict)=flush_grasp(objInput=objInputUnit,
                                          listener=listener,
                                          br=br,
                                          isExecute=isExecute,
                                          objId=TARGET,
                                          binId=binId,
                                          flag=flag,
                                          withPause = False)
                        
def placing_test(binId, flag, listener,br):
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
    
    (output_dict) = flush_grasp(objInput=objInput,
                                listener=listener,
                                br=br,
                                isExecute=isExecute,
                                objId=TARGET,
                                binId=binId,
                                flag=flag,
                                withPause = False,
                                rel_pose=rel_pose,
                                BoxBody=BoxBody,
                                place_pose=place_pose)
                        
                        
def visualize_proposals_test(listener,br):
    #~ viz_array_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    proposal_viz_array_pub = rospy.Publisher('/proposal_visualization_marker_array', MarkerArray, queue_size=10)
    rospy.sleep(0.5)
    proposals = np.zeros((3,5))
    proposals[0,:] = np.array([1,0,0.,0.1,.11 ])
    proposals[1,:] = np.array([.8,0,0.,0.1,.11 ])
    proposals[2,:] = np.array([1.2,0,0.0,.1,.11 ])
    
    visualize_flush_proposals(proposal_viz_array_pub, proposals, 0, listener, br,  is_selected = False)
    
            
# To test the function
if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.5)
    unit_test()
    #~ visualize_proposals_test(listener,br)
    #~ placing_test(binId = 7, flag = 2, listener=listener, br=br)

