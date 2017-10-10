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

import rospy
import ik.helper
import ik.ik

def goToHome(isExecute = True, withPause = False, slowDown = False):

    ik.ik.setAcc(4,4)
    if slowDown:
        ik.ik.setSpeedByName(speedName = 'fastest')
    else:
        ik.ik.setSpeed(1200,600)
    
    
    home_joint_pose = [-0.005744439031, -0.6879946105, 0.5861570764, 0.09693312715, 0.1061231983, -0.1045031463]
    planner = ik.ik.IKJoint(target_joint_pos=home_joint_pose)
    plan = planner.plan()
    print ('[goToHome]')
    
    plan.visualize()
    if isExecute:
        ik.helper.pauseFunc(withPause)
        plan.execute()
    
    return True
    
def goToARC(isExecute = True, withPause = False, slowDown = False):
    
    ik.ik.setAcc(4,4)
    if slowDown:
        ik.ik.setSpeedByName(speedName = 'fastest')
    else:
        ik.ik.setSpeed(1200,600)
    
    home_joint_pose = [-0.0014,    0.2129,    0.3204,    0,    1.0374,   -0.0014]
    planner = ik.ik.IKJoint(target_joint_pos=home_joint_pose)
    plan = planner.plan()
    print ('[goToARC]')
    
    plan.visualize()
    if isExecute:
        ik.helper.pauseFunc(withPause)
        plan.execute()
    
    return True

if __name__=="__main__":
    ## initialize listener rospy
    rospy.init_node('listener', anonymous=True)
    
    goToHome(robotConfig=None,
            homePos = [1,0,1.2], 
            isExecute = True,
            withPause = False)

