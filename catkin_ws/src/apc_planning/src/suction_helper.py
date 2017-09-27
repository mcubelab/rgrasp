
import tf
import numpy as np
from manual_fit.srv import *
from pr_msgs.msg import *
import rospy
import sensor_msgs.msg

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
import time
import yaml


def start_suction():
    rospy.init_node('suction_helper')
    rospy.wait_for_service('suction_service',timeout = 5)
    suction_service=rospy.ServiceProxy('suction_service', SuctionData1)
    resp1=suction_service("son", "")

def stop_suction():
    rospy.init_node('suction_helper')
    rospy.wait_for_service('suction_service',timeout = 5)
    suction_service=rospy.ServiceProxy('suction_service', SuctionData1)
    resp1=suction_service("soff", "")
    
if __name__=="__main__":
    start()
    time.sleep(3)
    print 'hello!'
    stop()
