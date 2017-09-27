#!/usr/bin/env python

from manual_fit.srv import *
from pr_msgs.msg import *
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from visualization_msgs.msg import *

import rospy
from std_msgs.msg import Float64

def vaccuum_test():
    rospy.init_node('listener', anonymous=True)
    print rospy.wait_for_message("/vac_level", Float64)

if __name__=="__main__":
    vaccuum_test()