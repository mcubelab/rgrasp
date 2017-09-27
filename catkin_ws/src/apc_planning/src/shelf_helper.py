#!/usr/bin/env python
# suction down primitive:

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# # The vertical dimension of the object should be smaller than the maximum
# gap distance between the two fingers.

import tf
import numpy as np
from manual_fit.srv import *
from pr_msgs.msg import *
import rospy
import scorpion


if __name__=="__main__":
    
    rospy.init_node('suction_down_simple')
    
import sensor_msgs.msg

from pr_msgs.srv import SuctionData1
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from visualization_msgs.msg import *
import copy
from collision_detection.collisionHelper import *

from rospy import ROSException
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import IKGuarded
from ik.ik import SuctionGuard
from ik.ik import WeightGuard
from ik.ik import getCart
from gripper import close
from ik.ik import fastfk
from ik.ik import setAcc

# put shared function into ik.helper module
from ik.helper import pauseFunc
from ik.helper import quat_from_matrix
import tf.transformations as tfm
from std_msgs.msg import Float64
from std_msgs.msg import Bool

#import suction_projection
from suction_projection import suction_projection_func
from suction_projection import optimize_angle
import suction
from ik.helper import pose_transform_precise_placing
from collision_detection.collisionHelper import *
import suction_down_simple
import matplotlib.pyplot as plt

def open_shelf(withPause=False):
    bin_id=0
    suc_down_orientation = [0.0, 1.0, 0.0,0.0]
    #theta=np.pi
    theta=0.0
    orient_mat_4x4 =[[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]]
    suc_down_orientation=quat_from_matrix(np.dot(orient_mat_4x4,tfm.quaternion_matrix(suc_down_orientation)))
    tip_hand_transform = [0, 0, .45, 0,0,0]

    #rospy.init_node('suction_down_simple', anonymous=True)
    joint_topic = '/joint_states'
    rate = rospy.Rate(10) # 10hz
    
    #Prevent collision
    scorpion.back()
    suction.electromagnet_start()
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg()
        q0 = APCrobotjoints.position
        
        if len(q0) >= 6:
            q0 = q0[0:6]   # take first 6, because in virtual environment there will be additional 2 hand joint
            break

    #q0=[q0[0],q0[1],q0[2],q0[3],q0[4],(-90.0*np.pi)/180]

    bin_pose = rospy.get_param('bin' + str(bin_id) + '_pose')
    bin_centroid = 1.0 * np.array([bin_pose['x'], bin_pose['y'], bin_pose['z']])

    pre_basepos = fastfk(q0)

    if pre_basepos==False or pre_basepos==None:
        pre_basepos=suction_position_base
    else:
        pre_basepos=[bin_centroid[0],pre_basepos[1],bin_centroid[2]+.2]

    x_offset=0.0
    pos0=[bin_centroid[0]+x_offset,-.05,bin_centroid[2]+.2]
    pos1=[bin_centroid[0]+x_offset,-.05,bin_centroid[2]-.01]
    pos2=[bin_centroid[0]+x_offset,.2,bin_centroid[2]-.01]
    pos3=[bin_centroid[0]+x_offset,.445,bin_centroid[2]-.01]
    pos4=[bin_centroid[0]+x_offset,.3,bin_centroid[2]+.1]
    input_speed_list=['faster']*10

    position_list=[pre_basepos,pos0,pos1,pos2,pos3,pos4]
    qf=q0
    qf_list=[]
    plans=[]
    success_flag=True
    for i in range(0, len(position_list)):
        # for targetPosition in position_list:
        #desiredPose=poseTransform(targetPosition, 'suction_shelf','map', listener)
        #compute the plan that moves the robot from pose q0 to pose [target_tip_pos,target_tip_ori]
        #specifically, we are moving the suction cup, whose relative location to link6 is determined by tip_hand_transform

        targetPosition = position_list[i]

        planner = IK(q0 = qf, target_tip_pos = targetPosition, target_tip_ori = suc_down_orientation,
                     tip_hand_transform=tip_hand_transform, joint_topic='/joint_states' , useFastIK = True, weight = [1,1,1,2,1,10])
        plan = planner.plan()
        #if  IK works for the current step, append it to the plan list, otherwise break
        if plan.success():  # whether or not IK actually worked
            plan.setSpeedByName(input_speed_list[i])
            plans.append(plan)
        else:
            success_flag=False
            break

        #the current final position of the plan sequence is stored in qf
        qf = copy.deepcopy(plan.q_traj[len(plan.q_traj)-1])
        #print qf
        qf_list.append(qf)

    print success_flag
    if not success_flag:
        return {"success_flag": success_flag}
        
    for plan in plans:
        plan.visualize(hand_param=0) # visualize  this step in RViz
        pauseFunc(withPause) # this makes it so that you have to press enter before the plan will run
        setAcc(4,4)
        plan.execute() #this executes a given plan (move from joint angle A to angles B associated with that plan)

if __name__=="__main__":
    #print suction_down_simple.suction_down_simple(withPause=True,suction_position_target=[-3,-5,-5],surface_normal=[0,0,1.0],flag=1,bin_id=0,print_messages=False,obj_ID='expo_eraser',rel_pose=None,BoxBody=None,place_pose=None,box_height=0.1)
    open_shelf(withPause=False)

