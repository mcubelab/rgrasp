#!/usr/bin/env python
# goToHome primitive:

# description: 
# called when we want to go back home, BE VERY CAREFUL IF CALLED INCORRECTLY
# OR AT A BAD TIME IT MAY HIT THE CAMERAS OR THE SHELF, designed to be used
# from mouth of bins or from the objective bin, not when hand is inside the bin

# the robot moves in essentially a straight line from where it is to the home
# tcp pose (position is taken as argument, pose is assumed to be gripper open
# toward shelf)

# inputs:
# configuration of the robot when called, assumed to be at the mouth bin or
# at objective bin
# location of the home, xyz in world coordinate frame and orientation, this
# is subject to change

import geometry_msgs.msg
import std_msgs
import json
import tf
import rospy
import pdb
import rospy
import numpy as np
import math
import tf.transformations as tfm
import gripper
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc, visualizeFunc, getObjCOM, openGripper, closeGripper
import threading
import spatula
import scorpion
import suction
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK, IKJoint, Plan, generatePlan, executePlanForward, setSpeedByName, setSpeed, setAcc

def goToHome(isExecute = True, withPause = False, slowDown = False):

    setAcc(4,4)
    if slowDown:
        setSpeedByName(speedName = 'fastest')
    else:
        setSpeed(1200,600)
    
    
    home_joint_pose = [-0.005744439031, -0.6879946105, 0.5861570764, 0.09693312715, 0.1061231983, -0.1045031463]
    planner = IKJoint(target_joint_pos=home_joint_pose)
    plan = planner.plan()
    print ('[goToHome]')
    
    plan.visualize()
    if isExecute:
        pauseFunc(withPause)
        plan.execute()
    
    return True
    
def goToARC(isExecute = True, withPause = False, slowDown = False):
    
    setAcc(4,4)
    if slowDown:
        setSpeedByName(speedName = 'fastest')
    else:
        setSpeed(1200,600)
    
    home_joint_pose = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    planner = IKJoint(target_joint_pos=home_joint_pose)
    plan = planner.plan()
    print ('[goToARC]')
    
    plan.visualize()
    if isExecute:
        pauseFunc(withPause)
        plan.execute()
    
    return True
    
#~ def goToHomeAndCalibrate():
    #~ thread = threading.Thread(target=spatula.home)
    #~ thread.start()
    #~ goToHome(isExecute = True, withPause = False, slowDown = False)
    #~ thread.join()

def get_safe_plan():
    from ik.ik import IKJoint, GetJoints
    q0 = GetJoints('/joint_states')
    qf_list = list(q0)
    qf_list[5] = 0
    qf = tuple(qf_list)
    safe_move = IKJoint(target_joint_pos=qf, q0=q0)
    safe_plan = safe_move.plan()
    safe_plan.setSpeedByName('gripperRotation')
    return safe_plan

def prepGripperPicking():
    print '[prepGripperPicking]'
    safe_plan = get_safe_plan()
    safe_plan.execute() # Move safe pos for moving scorpion
    spatula.open() #~open spatula
    gripper.close() #~close gripper
    scorpion.back() #~open scorpion
    safe_plan.executeBackward() # Move back to original position
    
def prepGripperSuction():
    print '[prepGripperSuction]'
    safe_plan = get_safe_plan()
    safe_plan.execute() # Move safe pos for moving scorpion
    spatula.open() #~open spatula
    gripper.close() #~close gripper
    scorpion.fwd() #~open scorpion
    safe_plan.executeBackward() # Move back to original position

if __name__=="__main__":
    ## initialize listener rospy
    rospy.init_node('listener', anonymous=True)
    
    goToHome(robotConfig=None,
            homePos = [1,0,1.2], 
            isExecute = True,
            withPause = False)

