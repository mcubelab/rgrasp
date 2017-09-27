#!/usr/bin/env python

# a minimal example of using python interface for drake
import geometry_msgs
import std_msgs
import json
from ik.roshelper import ROS_Wait_For_Msg
from ik.helper import pause
from ik.ik import IK
from ik.ik import activateCSS, deactivateCSS
import ik.ik
import tf.transformations as tfm
import rospy
import pdb
import sys
import tf
import math
import numpy as np
import tf.transformations as tfm
prepare_pose = [1.3, 0, 1.04,  0, 0.7071, 0, 0.7071]  # try to push the second bar
target_pose = [1.8, 0, 1.04,  0, 0.7071, 0, 0.7071]
target_pose = [1.3, 0, 0.9,  0, 0.7071, 0, 0.7071]


# prepare_pose = [1.3, 0, 0.8,  0, 0.7071, 0, 0.7071]  # try to push the second bar
# target_pose = [1.8, 0, 1.04,  0, 0.7071, 0, 0.7071]
# target_pose = [1.3, 0, 0.65,  0, 0.7071, 0, 0.7071]

l2 = 0.12 + 0.34  # 0.12: gripper base length, 0.34: gripper base to finger tip
tip_hand_transform = [0, 0, l2, 0,0,0] # x,y,z,r,p,y

rospy.init_node('testik', anonymous=True)
joint_topic = '/joint_states'

planner = IK(target_tip_pos = prepare_pose[0:3], target_tip_ori = prepare_pose[3:7], 
             joint_topic=joint_topic, tip_hand_transform=tip_hand_transform) 
plan1 = planner.plan()
plan1.setSpeedByName('slow')  # better start with slow speed

if plan1.success():
    print 'success'
    plan1.visualize()
    pause()
    plan1.execute()
else:
    print 'failed'
    sys.exit()

planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], 
             joint_topic=joint_topic, tip_hand_transform=tip_hand_transform) 
plan = planner.plan()
plan.setSpeedByName('slow')  # better start with slow speed

## visualize the frame
br = tf.TransformBroadcaster()
rospy.sleep(0.5)
br.sendTransform(tuple(tip_hand_transform[0:3]), 
                 tfm.quaternion_from_euler(*tip_hand_transform[3:6]), 
                 rospy.Time.now(), 'tip', "link_6")
br.sendTransform(tuple(target_pose[0:3]), tuple(target_pose[3:7]), 
                 rospy.Time.now(), 'target_pose', "world")

# get user input to visualize again or execute
user_reply=raw_input('Execute? [(y)es/(n)o]: ')
if user_reply == 'y':
    # the following parameters are the default parameters for activateCSS. It is the same as activateCSS() 
    activateCSS(refFrame = ik.ik.CSS_REFFRAME_TOOL, softDir=ik.ik.CSS_X, stiffness = 0, 
                stiffnessNonSoftDir = 50, allowMove = True, ramp = 100, refOrient = (0,0,0,1))  
    pause()
    plan.execute()
    pause()
    plan.executeBackward()
    
    # If pose is None, then the target pose will be at the current pose
    # You can specify pose as [x,y,z, qx,qy,qz,qw], the tcp pose we want to move to after CSS ended. x,y,z in meter.
    deactivateCSS(pose = None)

