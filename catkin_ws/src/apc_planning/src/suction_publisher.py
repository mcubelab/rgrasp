#!/usr/bin/env python
# suction down primitive:

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# # The vertical dimension of the object should be smaller than the maximum
# gap distance between the two fingers.

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

haveraspberry = rospy.get_param('/use_raspberry', True)

def read_vac_level():
    if haveraspberry:
        try:
            msg = rospy.wait_for_message("/rpi/vac_level" , Float64, 3)
            return msg.data
        except:
            print("Cannot read vac level")
            return 0
    else:
        #print 'Suction not connected!'
        return 0

def read_suction_status():
    if haveraspberry:
        try:
            msg = rospy.wait_for_message("/rpi/suction_status" , Bool, 3)
            return msg.data
        except:
            print("Cannot read suction_status")
            return None
    else:
        #print 'Suction not connected!'
        return None

def publish_vac_level():

    pub = rospy.Publisher('vac_engaged', Bool, queue_size=3)
    rate = rospy.Rate(10) # 10hz

    haverobot = rospy.get_param('/have_robot', False)
    suction_history=[]
    max_len=10

    time1 = time.time()
    duration = 0
    toff=0

    engaged_status = None
    bad_data_counter=0

    while not rospy.is_shutdown():
        current_level = read_vac_level()
        suction_status = read_suction_status()

        duration = time.time() - time1

        if suction_status==False:
            suction_history=[]
            toff=duration
            engaged_status = False
        elif suction_status==None:
            engaged_status = None
        else:
            suction_history.append(current_level)
            current_level=np.mean(np.array(suction_history))
            if len(suction_history)>max_len:
                suction_history.pop(0)

        offtime=duration-toff
        threshold_level_YES = 925.5
        threshold_level_NO  = 926.5


        if threshold_level_YES >= threshold_level_NO:
            print '******** Threshold wrong!! ********' # Make sure threshold_level_YES is lower than threshold_level_NO

        if abs(current_level-1000)<500 and suction_status:
            if current_level<threshold_level_YES:
                engaged_status=True
            elif current_level>threshold_level_NO:
                engaged_status = True
            #else:
                #engaged_status = None

        if abs(current_level-1000)<500 or not suction_status:
            pub.publish(engaged_status)
            bad_data_counter=0

        #If suction is on and it is still reading bullshit data
        #increment the bad_data_counter
        if not (abs(current_level-1000)<500 or not suction_status):
            bad_data_counter=bad_data_counter+1

        if bad_data_counter>6:
            pub.publish(None)
            bad_data_counter=0
        #print engaged_status



def get_vac_level(run_real):
    #rospy.init_node('listener', anonymous=True)
    if run_real:
        try:
            # wait for publisher for 1 seconds.
            wait_time = 1
            vac_out=rospy.wait_for_message("/rpi/vac_level", Float64, wait_time)
            # return vac_out<950
            return vac_out.data
        except:
            return None
    else:
        return None



if __name__=="__main__":
    rospy.init_node('suction_publisher')
    try:
        publish_vac_level()
    except rospy.ROSInterruptException:
        pass
