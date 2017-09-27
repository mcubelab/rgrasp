#!/usr/bin/python

import sys
import os
sys.path.append(os.environ['APC_BASE']+"/catkin_ws/src/apc_planning/src/")

import rospy
rospy.init_node('pythonapc', anonymous=True)
import suction
import gripper
import spatula
import suctionflip
from ik.roshelper import *
from ik.helper import *
from apc.helper import *
from ik.ik import *
import tf.transformations as tfm
import numpy as np


listener = tf.TransformListener()
br = tf.TransformBroadcaster()
rospy.sleep(0.5)

print "pubFrame(br, pose, 'obj', 'map')"


import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *

setJoints = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)   # should move service name out
setCart(pos=[1.012, 0.043, 0.418], ori=[0, -1, 0, 0 ])
#setCart(pos=[1.012, 0.043, 0.318], ori=[0, -1, 0, 0 ])
#setJoints (2.43, 34.63,19.37,0.0,36.01,2.43)
#setJoints (2.43, 40.23,20.28,0.0,29.49,2.43)


#exit(0)

# We create a list of two plans
plans = []
p1 = IKGuarded(target_tip_pos=[1.012, 0.043, 0.318], target_tip_ori=[0, -1, 0, 0 ], tip_hand_transform=[0.0, 0.0, 0, 0, 0, 0]).plan()
plans.append(p1)

p2 = IKGuarded(target_tip_pos=[1.012, 0.043, 0.218], target_tip_ori=[0, -1, 0, 0 ], tip_hand_transform=[0.0, 0.0, 0, 0, 0, 0], q0 = p1.q_traj[-1]).plan()
plans.append(p2)

succExec = True
end_index = 0

# enumerate() spit out both index and value of an list

for iplan, plan in enumerate(plans):   
    
    # Remember where we end in the plans
    end_index = iplan
    
    # succExec will be False if there is a motion supervision during execution
    succExec = plan.execute()
    
    print 'iplan', iplan, 'succExec', succExec
    
    if not succExec:
        print 'Plan execution ends due to motion supervision.'
        pause()
        break
        

print 'plan[0].j_stopped', plans[0].j_stopped
plans[0].j_stopped -= 10
plans[0].j_stopped = max(plans[0].j_stopped,0)
print 'Ready to come back?'
pause()

successfulPlans = plans[ 0 : (end_index+1) ]
backPlans = reversed( successfulPlans )

for iplan, plan in enumerate( backPlans ):
    plan.executeBackward()

