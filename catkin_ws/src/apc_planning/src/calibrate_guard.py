#!/usr/bin/env python

import tf.transformations as tfm
import numpy as np
from ik.roshelper import *
from ik.helper import *
from ik.ik import *
import tf
import rospy
import numpy
from tf.broadcaster import TransformBroadcaster
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from apc_vision.srv import *
from ik.roshelper import pose2list
from visualization_msgs.msg import Marker
from mpl_toolkits.mplot3d import Axes3D
from apc_arduino.srv import ServiceBridge
from apc_arduino.msg import StrainGaugeData
from apc_arduino.msg import HallEffectData
import matplotlib.pyplot as plt
import tf.transformations as tfm
from ik.ik import *
import goToHome
import gripper
import spatula
import time
import suction_cup_ori
import suctionflip
import yaml
import optparse

arduino_service = rospy.ServiceProxy('/Arduino/Arduino_Services', ServiceBridge)

def TuneSG(sg_no = -1):
    if sg_no == -1: arduino_service(3,5,0)
    elif sg_no < 4 and sg_no > -1: arduino_service(3,sg_no,0)

approach_pose_jnt = [28.3,11.72,44.65,0.0,33.64,-151.7]
approach_pose_jnt_rad = [n*np.pi/180.0 for n in approach_pose_jnt ]
approach_pos_cart = [0.650,0.350,0.500]
approach_pos_ori = [0.0,1.0,0.0,0.0]
finger1_approach = [0.650,0.350,0.500,0.9238, 0.38268,0.0,0.0]
finger2_approach = [0.650,0.350,0.500,0.38268, -0.9238, 0.0, 0.0]
approach_pos = [0.650,0.350,0.500]
touch_distance = 20

ori1 = [0.9238, 0.38268,0.0,0.0]
ori2 = [0.38268, -0.9238, 0.0,0.0]

safety_range_HE = 10.0
safety_range_SG = 3.0
gripperOri=[0, 1, 0, 0]

finger1_sg = 3
finger2_sg = 1 

def init():
    goToHome.goToHome()
    gripper.open()
    spatula.open()
    
def read_HE_sensors():
    try:
        
        msg = rospy.wait_for_message("/Arduino/HEData" , HallEffectData,5)     
        halleffect_0 = msg.HE0
        halleffect_1 = msg.HE1
        halleffect_2 = msg.HE2
        halleffect_3 = msg.HE3
    except:
        return [0,0,0,0]
    return [halleffect_0, halleffect_1,halleffect_2, halleffect_3]
    
def read_Strain_Gauge(sg_no):
    import pdb;pdb.set_trace()
    straingauge= 0.0
    try:
        msg = rospy.wait_for_message("/Arduino/StrGData" , StrainGaugeData,5)     
        if sg_no == 0: straingauge = msg.strain0
        elif sg_no == 1: straingauge = msg.strain1
        elif sg_no == 2: straingauge = msg.strain2
        elif sg_no == 3: straingauge = msg.strain3
    except:
        print "[Calibrate Suction Guard] wait for message did't work"
        return 0.0
    return straingauge

def calib_HE():
    
    goToHome.goToHome()
    
    reading_straight_arr = []
    reading_side_arr = []
   
    suctionflip.suctionflip(True, False,0,0,True,False)
    for num in range(0,100):
        reading_straight_arr.append(read_HE_sensors())
    suctionflip.suctionflip(True, False,1,1,True,False)
    for num in range(0,100):
        reading_side_arr.append(read_HE_sensors())
    print reading_side_arr
    reading_straight_min = np.amin(reading_straight_arr,axis = 0).tolist()
    reading_straight_max = np.amax(reading_straight_arr,axis = 0).tolist()
    
    HE0 = reading_straight_max[0] - reading_straight_min[0] + safety_range_HE
    HE2 = reading_straight_max[2] - reading_straight_min[2] + safety_range_HE
    
    reading_side_min = np.amin(reading_side_arr,axis = 0).tolist()
    reading_side_max = np.amax(reading_side_arr,axis = 0).tolist()
    
    HE1 = reading_side_max[1] - reading_side_min[1] + safety_range_HE
    HE3 = reading_side_max[3] - reading_side_min[3] + safety_range_HE
    ################### WRITE DATA ####################
    f1path = rospy.get_param("Finger1_Path")
    stream1 = open(f1path, 'r')
    data1 = yaml.load(stream1)
    f2path = rospy.get_param("Finger2_Path")
    stream2 = open(f2path, 'r')
    data2 = yaml.load(stream2)
    data1['/arduino_params/HE0_Contact_Threshold'] = HE0
    data1['/arduino_params/HE1_Contact_Threshold'] = HE1
    with open(f1path, 'w') as yaml_file:
        yaml_file.write( yaml.dump(data1, default_flow_style=False))
    print("Suction Sensor 1 calibrated. Parameters written")
    data2['/arduino_params/HE2_Contact_Threshold'] = HE2
    data2['/arduino_params/HE3_Contact_Threshold'] = HE3
    with open(f2path, 'w') as yaml_file:
        yaml_file.write( yaml.dump(data2, default_flow_style=False))
    print("Suction Sensor 2 calibrated. Parameters written.")
    
    
    
def calib_straingauges():
    init()

    goToHome.goToHome()
    
    reading_SG1_arr = []
    reading_SG2_arr = []
    TuneSG(finger1_sg)
    for num in range(0,30):
        reading_SG1_arr.append(read_Strain_Gauge(finger1_sg))
        print num
    TuneSG(finger2_sg)
    for num in range(0,30):
        reading_SG2_arr.append(read_Strain_Gauge(finger2_sg))
        print num

    reading_SG1_min = np.amin(reading_SG1_arr,axis = 0).tolist()
    reading_SG1_max = np.amax(reading_SG1_arr,axis = 0).tolist()
    
    SG1  = reading_SG1_max - reading_SG1_min + safety_range_SG
    
    reading_SG2_min = np.amin(reading_SG2_arr,axis = 0).tolist()
    reading_SG2_max = np.amax(reading_SG2_arr,axis = 0).tolist()
    
    SG2  = reading_SG2_max - reading_SG2_min + safety_range_SG
    
    ################### WRITE DATA ####################
    f1path = rospy.get_param("Finger1_Path")
    stream1 = open(f1path, 'r')
    data1 = yaml.load(stream1)
    f2path = rospy.get_param("Finger2_Path")
    stream2 = open(f2path, 'r')
    data2 = yaml.load(stream2)
    data1['/arduino_params/Strain_Gauge_F1_Threshold'] = SG1 
    with open(f1path, 'w') as yaml_file:
        yaml_file.write( yaml.dump(data1, default_flow_style=False))
    print("Strain Gauge 1 calibrated. Parameters written")
    data2['/arduino_params/Strain_Gauge_F2_Threshold'] = SG2
    with open(f2path, 'w') as yaml_file:
        yaml_file.write( yaml.dump(data2, default_flow_style=False))
    print("Strain Gauge 2 calibrated. Parameters written.")
    

    

if __name__=="__main__":
    parser = optparse.OptionParser()
    parser.add_option('-e', '--calibHE', action="store_true", dest='tocalibsuc', 
                          help='To calibrate suction guard or not', 
                      default=False)  
    parser.add_option('-H', '--testHE', action="store_true", dest='totestsuc', 
                          help='To test suction guard or not', 
                      default=False)  

    parser.add_option('-s', '--calibstraingauge', action="store_true", dest='tocalibSG', 
                          help='To calibrate strain gauge guard guard or not', 
                      default=False)  
    parser.add_option('-S', '--testSG', action="store_true", dest='toteststrain', 
                          help='To test suction guard or not', 
                      default=False)  

    (opt, args) = parser.parse_args()
    rospy.init_node("guard_calib")
    if opt.tocalibsuc:
        calib_HE()
    elif opt.tocalibSG:
        calib_straingauges()
    elif opt.totestsuc:
        raw_input("Test suction side guard. Make sure suction cups are pointing to the side. Press enter ...")
        sucsideGuard = SideSuctionGuard()
        sucsideGuard.prep()
        for i in range (20):
            print "Test",i,":", sucsideGuard.test()
            rospy.sleep(.5)
        raw_input("Test suction straight guard. Make sure suction cups are pointing straight.Press enter ...")
        sucstrGuard = StraightSuctionGuard()
        sucstrGuard.prep()
        rospy.sleep(1)
        for i in range (20):
            print "Test",i,":", sucstrGuard.test()
            rospy.sleep(.5)
    elif opt.toteststrain:
        raw_input("Test strain gauge guard for finger 1. Make sure spatulas are open. Press enter ...")
        sgGuard = strainGaugeGuard(1)
        sgGuard.prep()
        for i in range (20):
            print "Test",i,":", sgGuard.test()
            rospy.sleep(.5)
        raw_input("Test strain gauge guard for finger 2. Make sure spatulas are open. Press enter ...")
        sgGuard = strainGaugeGuard(2)
        sgGuard.prep()
        for i in range (20):
            print "Test",i,":", sgGuard.test()
            rospy.sleep(.5)
        
    else:
        print read_HE_sensors()

