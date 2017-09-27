#!/usr/bin/python

import sys
import os
sys.path.append(os.environ['APC_BASE']+"/catkin_ws/src/apc_planning/src/")

import rospy
rospy.init_node('pythonapc', anonymous=True)
from ik.roshelper import *
from ik.helper import *
from apc.helper import *
from ik.ik import *
import tf.transformations as tfm
import numpy as np

# set cart to some initial pose (not needed for primitives)
setCart(pos=[1.012, 0.043, 0.418], ori=[0, -1, 0, 0 ])

# We create a list of two plans using IKGuarded(); IK() should also work
plans = []
p1 = IKGuarded(target_tip_pos=[1.012, 0.043, 0.318], target_tip_ori=[0, -1, 0, 0 ], tip_hand_transform=[0.0, 0.0, 0, 0, 0, 0]).plan()
plans.append(p1)

p2 = IKGuarded(target_tip_pos=[1.012, 0.043, 0.218], target_tip_ori=[0, -1, 0, 0 ], tip_hand_transform=[0.0, 0.0, 0, 0, 0, 0], q0 = p1.q_traj[-1]).plan()
plans.append(p2)

succExec = True
end_index = 0

# 1. Execute the plan in plans forward 

#    enumerate() will yield both index and value of a list
for iplan, plan in enumerate(plans):  
    
    # 1.1 Update where we end executing in the plans
    end_index = iplan
    
    # 1.2 succExec will be False if there is a motion supervision during execution
    #     the robot will move back a small step for IKGuarded or to the beginning of an IK plan.
    succExec = plan.execute()
    
    print 'iplan', iplan, 'succExec', succExec
    
    # 1.3 Abort the forward execution
    if not succExec:
        print 'Plan execution ends due to motion supervision.'
        # rospy.sleep(5)  # not sure if we need some long sleep
        break

print "Let's come back"
#pause()

# 2. Go back from end_index

# 2.1 Get the portion of plans that's been executed, including the failed one
successfulPlans = plans[ 0 : (end_index+1) ] 

# 2.2 Reverse those plans
backPlans = reversed( successfulPlans )

# 2.3 Run them backwards
for iplan, plan in enumerate( backPlans ):
    plan.executeBackward()

