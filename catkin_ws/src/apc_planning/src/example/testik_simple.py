#!/usr/bin/env python

# a minimal example of using python interface for drake
import geometry_msgs
import std_msgs
import json
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
import tf.transformations as tfm
import rospy
import pdb
import sys
import tf
import math
import numpy as np
import tf.transformations as tfm
target_pose = [0.94, 0, 1.2965,  0, 0.7071, 0, 0.7071]
#target_pose = [0.70, 0-0.2, 1.2965,  0, 0.7071, 0, 0.7071]

l2 = 0.12 + 0.34  # 0.12: gripper base length, 0.34: gripper base to finger tip
tip_hand_transform = [0, 0, l2, 0,0,0] # x,y,z,r,p,y
shelf_world_transform = [1.7831, 0.0036694, -0.50575, 0, 0, 0.70711, 0.70711] # x,y,z,qx,qy,qz,qw
shelf_world_transform = [1.4026, -0.023559, -0.4461, 0, 0, 0.70711, 0.70711] # x,y,z,qx,qy,qz,qw
shelf_world_transform = [1.91, 0, -0.5, 0, 0, 0.70711, 0.70711] # x,y,z,qx,qy,qz,qw

rospy.init_node('listener', anonymous=True)
joint_topic = '/joint_states'


planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], 
             joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, 
             straightness = 0.3) # straightness: [0-1] at what point it should exactly follow the target orientation
plan = planner.plan()
#plan.setSpeedByName('fast')  # if speed of a plan is not set, the robot execution will follow the speed set previously

if plan.success():
    print 'success'
    plan.visualize()
else:
    print 'failed'
    sys.exit()

## visualize the frame
br = tf.TransformBroadcaster()
rospy.sleep(0.5)
br.sendTransform(tuple(tip_hand_transform[0:3]), 
                 tfm.quaternion_from_euler(*tip_hand_transform[3:6]), 
                 rospy.Time.now(), 'tip', "link_6")
br.sendTransform(tuple(target_pose[0:3]), tuple(target_pose[3:7]), 
                 rospy.Time.now(), 'target_pose', "world")

# get user input to visualize again or execute
while True:
    user_reply=raw_input('Execute? [(y)es/execute (b)ackward/(v)isualize/visualize b(a)ckward/(e)nd]: ')
    if user_reply == 'y':
        plan.execute()
    if user_reply == 'b':
        plan.executeBackward()
    elif user_reply == 'v':
        plan.visualize()
    elif user_reply == 'a':
        plan.visualizeBackward()
    elif user_reply == 'e':
        break

