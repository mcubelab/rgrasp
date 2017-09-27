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

def read_vac_level():
    try:
        msg = rospy.wait_for_message("/rpi/vac_level" , Float64, 3)
        vac_level = msg.data
    except:
        print("Cannot read vac level")
        return 0
    return vac_level
    
def read_flow_level():
try:
    msg = rospy.wait_for_message("/flow_level" , Float64, 3)
    vac_level = msg.data
except:
    print("Cannot read vac level")
    return 0
return vac_level
 
def read_suction_status():
    try:
        msg = rospy.wait_for_message("/rpi/suction_status" , Bool, 3)
        suction_status = msg.data
    except:
        print("Cannot read suction_status")
        return False
    return suction_status

def calibrate_vac_level():
    fig1 = plt.figure(1)
    
    plt.figure(1)
    plt.ion()
    ax1=plt.axes()
    
    #pub = rospy.Publisher('vac_engaged', Bool, queue_size=3)
    rate = rospy.Rate(10) # 10hz
    
    suction_history=[]
    max_len=10
    
    haveraspberry = rospy.get_param('/use_raspberry', True)
    haverobot = rospy.get_param('/have_robot', False)
    
    time1 = time.time()
    duration = 0
    toff=0
    
    engaged_status = False
    bad_data_counter=0
    
    count=0
    while duration<90:
        count=count+1
        
        if haverobot:
            current_level=read_flow_level()
            suction_status = read_suction_status()
        else:
            current_level=0
            suction_status = False
            
        duration = time.time() - time1

        if not suction_status:
            suction_history=[]
            toff=duration
            engaged_status = False
        else:
            suction_history.append(current_level)
            current_level=np.mean(np.array(suction_history))
            if len(suction_history)>max_len:
                suction_history.pop(0)
            

        offtime=duration-toff    
        #threshold_level=max(min(905.0,770.0+4.22*offtime),783.0)
        threshold_level=925.5
        if abs(current_level-1000)<500 and suction_status:
            engaged_status=current_level<threshold_level
        
        if abs(current_level-1000)<500 or not suction_status:
            #pub.publish(engaged_status)
            bad_data_counter=0
            
        #If suction is on and it is still reading bullshit data
        #increment the bad_data_counter    
        if not (abs(current_level-1000)>=500 or not suction_status):
            bad_data_counter=bad_data_counter+1
            
        if bad_data_counter>6:
            #pub.publish(None)
            bad_data_counter=0
        #print engaged_status 
        #plt.scatter(duration,threshold_level,color='b')
        plt.scatter(duration,threshold_level,color='b')
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
