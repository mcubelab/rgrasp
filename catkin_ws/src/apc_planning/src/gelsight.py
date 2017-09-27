#!/usr/bin/env python

import rospy
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from visualization_msgs.msg import *

# put shared function into ik.helper module

from std_msgs.msg import Float64

def lights_on():
    rospy.init_node('gelsight_helper')
    rospy.wait_for_service('suction_service',timeout = 5)
    suction_service=rospy.ServiceProxy('suction_service', SuctionData1)
    resp1=suction_service("lon", "")

def lights_off():
    rospy.init_node('gelsight_helper')
    rospy.wait_for_service('suction_service',timeout = 5)
    suction_service=rospy.ServiceProxy('suction_service', SuctionData1)
    resp1=suction_service("loff", "")

if __name__=="__main__":
    print 'hello!'
