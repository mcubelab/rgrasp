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
import yaml
import time
from suction_down_simple import calibrate_flow_sensor
from std_msgs.msg import Bool
from std_srvs.srv import Empty
haveraspberry = rospy.get_param('/use_raspberry', True)
haverobot = rospy.get_param('/have_robot', False)
    
def read_vac_level():
    if haveraspberry:
        try:
            msg = rospy.wait_for_message("/rpi/vac_level" , Float64, 3)
            return msg.data
        except:
            rospy.logwarn("Cannot read vac level")
            return 0
    else:
        #print 'Suction not connected!'
        return 0
        
def read_flow_level():
    try:
        msg = rospy.wait_for_message("/flow_level" , Float64, 3)
        vac_level = msg.data
        return vac_level
    except:
        rospy.logwarn("Cannot read vac level")
        return 100
    
    
def read_suction_status():
    #~ if haverobot:
    try:
        msg = rospy.wait_for_message("/robot1_SuctionState" , Bool, 3)   #Todo: Add this 
        return msg.data
    except:
        rospy.logwarn("Cannot read suction_status")
        return None
    #~ else:
        #~ #print 'Suction not connected!'
        #~ return True   #hack for test

calibrate_fs = rospy.ServiceProxy('flow_sensor_calibration', Empty)

def pre_calibrate_flow_sensor():
    #~ if haverobot:
    try:
        pre_calibrate_fs()
        rospy.loginfo('[Suction] Flow_sensor buffer emptied')
    #~ else:
    except:
        rospy.logwarn('[Suction] Flow_sensor not connected, skipping pre_calibrate_flow_sensor')

pre_calibrate_fs = rospy.ServiceProxy('flow_sensor_empty_buffer', Empty)
def calibrate_flow_sensor():
    if haverobot:
        calibrate_fs()
        rospy.loginfo('[Suction] Flow_sensor calibration complete')
    else:
        rospy.logwarn('[Suction] Flow_sensor not connected, skipping calibrate_flow_sensor')
        

def calibrate_vac_level():
    calibrate_flow_sensor()
    
    fig1 = plt.figure(1)
    
    plt.figure(1)
    plt.ion()
    ax1=plt.axes()
    
    #pub = rospy.Publisher('vac_engaged', Bool, queue_size=3)
    pub0 = rospy.Publisher('/flow_calib_status', Bool, queue_size=3)
    pub = rospy.Publisher('/vac_engaged', Bool, queue_size=3)
    rate = rospy.Rate(10) # 10hz


    suction_history=[]
    max_len=10

    time1 = time.time()
    duration = 0
    toff=0
    calibration_status = False
    engaged_status = None
    bad_data_counter=0
    
    threshold_level_YES = 0.045
    threshold_level_NO = 0.0002
    #~ threshold_level_YES = 0.57
    #~ threshold_level_NO  = 0.562
    #~ threshold_level_YES = 0.008
    #~ threshold_level_NO = 0.0002
    if threshold_level_YES < threshold_level_NO:
        print '******** Threshold wrong!! ********' # Make sure threshold_level_YES is lower than threshold_level_NO
    max_threshold = 0.55
    count = 0
    while not rospy.is_shutdown():
        count=count+1
        current_level = read_flow_level()
        rospy.loginfo( 'current_level: %s',current_level)
        suction_status = read_suction_status()
        #~ suction_status = True  # hack
        duration = time.time() - time1
        suction_status_pre = suction_status

        if suction_status == False:
            suction_history=[]
            toff=duration
            engaged_status = False
            calibration_status = False
        elif suction_status==None:
            engaged_status = None
            calibration_status = False
        else:
            if suction_status_pre == False:
                pre_calibrate_flow_sensor()
            suction_history.append(current_level)
            current_level=np.mean(np.array(suction_history))
            if len(suction_history)>max_len:
                suction_history.pop(0)

            offtime=duration-toff
            if offtime>1.2 and not calibration_status:
                calibrate_flow_sensor()
                calibration_status = True
            #~ else:
                #~ calibration_status = False

            if abs(current_level) < 0.5 and calibration_status:
                if current_level>threshold_level_YES:
                    engaged_status=True
                elif current_level < threshold_level_NO:
                    engaged_status = False
                else:
                    engaged_status = None
            else: 
                engaged_status = None

        if abs(current_level)<0.5 or not suction_status:
            bad_data_counter=0
        
        
        
        pub0.publish(calibration_status)
        pub.publish(engaged_status)
        
        #~ rospy.loginfo( 'suction_status: %s',suction_status)
        #~ rospy.loginfo( 'calibration_status: %s',calibration_status)
        #~ rospy.loginfo( 'engaged_status: %s',engaged_status)
        #~ rospy.loginfo( '________________________________')

        #If suction is on and it is still reading bullshit data
        #increment the bad_data_counter
        if not (abs(current_level)>0.5 or not suction_status):
            bad_data_counter=bad_data_counter+1
        if bad_data_counter > 6:
            pub.publish(None)
            bad_data_counter=0


        #print engaged_status 
        #plt.scatter(duration,threshold_level,color='b')
        plt.scatter(duration,threshold_level_YES,color='b')
        plt.scatter(duration,current_level,color='r')
        
        if count%10==0:   
            plt.pause(0.01)
    #pause()
    raw_input()

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
    rospy.init_node('suction_calibration')
    
    try:
        calibrate_vac_level()
    except rospy.ROSInterruptException:
        pass
