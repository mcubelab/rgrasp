#!/usr/bin/env python

import rospy
import numpy as np
#from manual_fit.srv import *
from copy import deepcopy
import roslib; roslib.load_manifest("robot_comm")
#from robot_comm.srv import *
import tf
#from collision_free_placing import  collision_free_placing
from ik.ik import generatePlan, EvalPlan, WeightGuard, executePlanForward
from ik.helper import get_joints, mat2quat, get_params_yaml, reference_frames, drop_pose_transform
from collision_detection.collisionHelper import collisionFree
import gripper,  scorpion
import ik.helper

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
    rospy.loginfo('[Picking] Starting primitive with flag: %d' % flag)
    rospy.loginfo('[Picking] objInput %s', objInput)
    rospy.loginfo('[Picking] objId %s', objId)
    rospy.loginfo('[Picking] flag %s', flag)
    rospy.loginfo('[Picking] withPause %s', withPause)
    rospy.loginfo('[Picking] rel_pose %s', rel_pose)
    rospy.loginfo('[Picking] BoxBody %s', BoxBody)
    rospy.loginfo('[Picking] place_pose %s', place_pose)

    ########################
    ## Initialize values ##
    ########################
    q_initial = get_joints()
    bin_pose = get_params_yaml('bin'+str(binId)+'_pose')
    spatula_tip_to_tcp_dist=rospy.get_param("/gripper/spatula_tip_to_tcp_dist")
    delta_vision_pose = get_params_yaml('vision_pose_picking')
    vision_pos=np.array(bin_pose[0:3])+np.array(delta_vision_pose[0:3])
    plans = []
    graspPose=[]
    plan_possible = False
    execution_possible = False
    gripper_opening = 0.0
    
    def compose_output():
        return {'collision':False,'grasp_possible':False,'plan_possible':False,'execution_possible':False,'gripper_opening':[],'graspPose':[],'gelsight_data':[],'final_object_pose':[]}

    #########################
    ## Constant parameters ##
    #########################
    UpdistFromObject = 0.05 # Margin above object during "move to a location" 
    gripperOriHome=[0, 1, 0, 0]
    l1 = 0.0
    l2 = 0.0
    l3 = spatula_tip_to_tcp_dist
    tip_hand_transform = [l1, l2, l3, 0,0,0]
    vision_pos[2] = 0.17786528
    
    ###############################
    ## Picking primitive flag: 0 ##
    ###############################
    if flag==0: 
        
        #~Define reference frames
        world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = reference_frames(listener= listener, br=br)
        
        #~get grasp pose and gripper opening from vision
        if len(objInput)==12:
            graspPos, hand_X, hand_Y, hand_Z, grasp_width = ik.helper.get_picking_params_from_12(objInput)
        elif len(objInput)==7:
            graspPos, hand_X, hand_Y, hand_Z, grasp_width = ik.helper.get_picking_params_from_7(objInput, objId, listener, br)
        
        #~build gripper orientation matrix 3x3
        hand_orient_norm = np.vstack([hand_X,hand_Y,hand_Z])
        hand_orient_norm=hand_orient_norm.transpose()
    
        #~Grasp relocation script
        hand_orient_quat=mat2quat(hand_orient_norm)
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
        pregrasp_targetPosition = graspPos - hand_Z*UpdistFromObject
        pregrasp_targetPosition[2] = bin_pose[2] + 0.15
        plan, qf, plan_possible = generatePlan(q_initial, pregrasp_targetPosition, hand_orient_quat, tip_hand_transform, 'superSaiyan', plan_name = 'go_safe_bin')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
                        
        #~1. Move to to surface of bin
        pregrasp_targetPosition[2] = bin_pose[2] + 0.05
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
        
       #~4. Go down  
        grasp_targetPosition=deepcopy(graspPos)
        rospy.loginfo('[Picking] Grasp Target %s', grasp_targetPosition)
        
        #~5. sleep
        sleep_plan = EvalPlan('rospy.sleep(0.2)')
        plans.append(sleep_plan)
        
         #~6. perform guarded move down
        grasp_targetPosition[2] = bin_pose[2] -  rospy.get_param('/bin'+str(binId)+'/height') + 0.000 #~frank hack for safety
        if binId==0:
            grasp_targetPosition[2] = grasp_targetPosition[2] +0.005

        plan, qf, plan_possible = generatePlan(q_initial, grasp_targetPosition, hand_orient_quat, tip_hand_transform, 'faster', guard_on=WeightGuard(binId, threshold = 100), plan_name = 'guarded_pick')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~7. Close spatula
        grasp_plan = EvalPlan('spatula.close()')
        plans.append(grasp_plan)
        
        #~8. Close gripper
        grasp_plan = EvalPlan('helper.graspinGripper(%f,%f)'%(200,50))
        plans.append(grasp_plan)
        
        #~9. sleep
        sleep_plan = EvalPlan('rospy.sleep(0.2)')
        plans.append(sleep_plan)

        #~10. Move to a location above the bin
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
        if isExecute and plan_possible:
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
                
        elif not isExecute:
            execution_possible=plan_possible
    
        return compose_output()
       
    ###################
    ## Active vision ##
    ###################
    elif flag == 1:
        return compose_output()
        
    #############
    ## Placing ##
    #############
    elif flag == 2:
        #if primitive called without placing arguments, go to center of bin
        if rel_pose==None:
            drop_pose = get_params_yaml('bin'+str(binId)+'_pose')
            drop_pose[3:7] = gripperOriHome
        else:
            drop_pose = drop_pose_transform(binId, rel_pose, BoxBody, place_pose, viz_pub)
           
        #~ Adjust height according to weigth
        if is_drop:
            drop_offset = 0.15
        else:
            drop_offset = 0.002
        #~set drop pose target z position to be bottom of bin
        drop_pose[2] = bin_pose[2] - rospy.get_param('bin'+str(binId)+'/height') + drop_offset
        
        #~Predrop: go to top middle bin surface fast without guarded move
        predrop_pos=np.array(drop_pose[0:3])
        predrop_pos[2] = bin_pose[2] + 0.2
         
        ######################
        ##  Generate plans ##
        ######################
        #~move safe to placing bin number
        rospy.logdebug('[Picking] Collision free placing to binId %s and position %s', binId, predrop_pos)
#        if isExecute:
#            q_initial = collision_free_placing(binId, listener, obj_ID=objId, BoxBody=BoxBody, isSuction = False,with_object = True, hand_orientation=drop_pose[3:7],projected_target_position = predrop_pos, isReturn = True)
#            if update_command is not None:
#                update_command.execute()

        
        #~1.fast motion to bin surface
        plan, qf, plan_possible = generatePlan(q_initial, predrop_pos, drop_pose[3:7], tip_hand_transform, 'faster',  plan_name = 'go_bin_surface')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~2. Guarded move down slow
        plan, qf, plan_possible = generatePlan(q_initial, drop_pose[0:3], drop_pose[3:7], tip_hand_transform, 'fast',   guard_on=WeightGuard(binId), backwards_speed = 'superSaiyan', plan_name = 'guarded_drop')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~3. open gripper
        grasp_plan = EvalPlan('helper.moveGripper(0.110, 200)')
        plans.append(grasp_plan)
        
        #~4. open spatula
        grasp_plan = EvalPlan('spatula.open()')
        plans.append(grasp_plan)
        
        #~5. go to predrop_pos
        plan, qf, plan_possible = generatePlan(q_initial, predrop_pos[0:3], drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'predrop_pos')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()
            
        #~6. go to final pose
        final_pos = bin_pose[0:3]
        final_pos[2] = bin_pose[2] + 0.3
        plan, qf, plan_possible = generatePlan(q_initial, final_pos, drop_pose[3:7], tip_hand_transform, 'superSaiyan', plan_name = 'final_pose')
        if plan_possible:
            plans.append(plan)
            q_initial = qf
        else:
            return compose_output()

        #~ Execute plans
        if plan_possible:
            if isExecute:
                executePlanForward(plans, withPause)
            execution_possible = True
            return compose_output()
            


def unit_test(listener, br):
    # Initialize
    isExecute = True
    rospy.sleep(0.5)
#    data = posesrv('','')
    #~ goToHome.prepGripperPicking()

    #~define objId 
    TARGET = 'expo_eraser'
    
    #~define obj poses 
    objInput = []
    objInput.append(get_params_yaml('bin0_pose'))
    objInput.append(get_params_yaml('bin1_pose'))
    objInput.append(get_params_yaml('bin2_pose'))
#    
    #~unit test for flags 1 and 2 in bins 0 and 1 (with weight guard)
    flag_list = [0,1,2]
    bin_list = [0,1,2]
    objInput = [[1.0550236701965332, -0.40252241492271423, -0.018005974590778351, 0.0, 0.0, -1.0, 0.15000000596046448, 0.10999999940395355, -0.38268342614173889, -0.92387950420379639, 0.0, 0.22444444894790649]]
    binId = 0
    flag = 0
    bin_drop_id = binId
    (output_dict)=grasp(objInput=objInput[binId],
                                        listener=listener,
                                        br=br,
                                        isExecute=isExecute,
                                        objId=TARGET,
                                        binId=bin_drop_id,
                                        flag=flag,
                                        withPause = False)
    #~ bin_list = [0]
#    for binId in bin_list:
#        for flag in flag_list:
#            if flag==2:
#                bin_list_drop = [binId] 
#                for bin_drop_id in  bin_list_drop:
#                    (output_dict)=grasp(objInput=objInput[binId],
#                                        listener=listener,
#                                        br=br,
#                                        isExecute=isExecute,
#                                        objId=TARGET,
#                                        binId=bin_drop_id,
#                                        flag=flag,
#                                        withPause = False)
#                    gripper.close()
#            else:
#                #~call pick function without xtra placing arguments (flag:0,1)
#                (output_dict)=grasp(objInput[binId],
#                                    listener=listener,
#                                    br=br,
#                                    isExecute=isExecute,
#                                    objId=TARGET,
#                                    binId=binId,
#                                    flag=flag,
#                                    withPause = False)

# To test the function
if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)

    unit_test(listener=listener, br=br)


