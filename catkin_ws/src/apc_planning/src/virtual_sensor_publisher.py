#!/usr/bin/env python

import tf
import numpy as np
from manual_fit.srv import *
from pr_msgs.msg import *
import rospy
import sensor_msgs.msg
import matplotlib.pyplot as plt
from pr_msgs.srv import SuctionData1
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from visualization_msgs.msg import *
import copy
from collision_detection.collisionHelper import *

from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import getCart
from gripper import close

# put shared function into ik.helper module
from ik.helper import pauseFunc
from ik.helper import quat_from_matrix
import tf.transformations as tfm
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
import yaml
import time
import random

haveraspberry = rospy.get_param('/use_raspberry', True)
haverobot = rospy.get_param('/have_robot', False)

def publish_virtual():
    flow_level_pub = rospy.Publisher('/flow_level', Float64, queue_size=3)
    suction_state_pub = rospy.Publisher('/robot1_SuctionState', Bool, queue_size=3)
    rate = rospy.Rate(100) # 100hz
    init_time=time.time()
    while not rospy.is_shutdown():
        duration=time.time()-init_time
        # pub the flow_level
        level = random.uniform(0, 0.52)        
        print level
        
        # pub the suction_state
        if duration%6 < 3:
            suction_state = False
        else:
            suction_state = True
        print suction_state
        
        flow_level_pub.publish(level)
        suction_state_pub.publish(suction_state)

if __name__=="__main__":
    rospy.init_node('virtual_sensor_publisher')
    try:
        if not haverobot:
            print 1
            publish_virtual()
    except rospy.ROSInterruptException:
        pass
