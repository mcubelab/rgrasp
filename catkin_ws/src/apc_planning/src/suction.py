#!/usr/bin/env python

import rospy
from robot_comm.srv import robot_IOSignal

ErrorMessage = 'Suction cup not connected, skipping command: '
haverobot = rospy.get_param('/have_robot', True) 

def electromagnet_start():
    command = 'electromagnet_start'
    electromagnet_on=rospy.ServiceProxy('/robot1_IOSignal',robot_IOSignal)
    
    if haverobot:  
        electromagnet_on(1,1)
    else:
        print '[Suction],', ErrorMessage, command
        
def electromagnet_stop():
    command = 'electromagnet_stop'
    electromagnet_off=rospy.ServiceProxy('/robot1_IOSignal',robot_IOSignal)
    
    if haverobot:
        electromagnet_off(1,0)
    else:
        print '[Suction],', ErrorMessage, command

def start():
    command = 'start'
    suction_on=rospy.ServiceProxy('/robot1_IOSignal',robot_IOSignal)
    
    if haverobot:  
        suction_on(2,1)
    else:
        print '[Suction],', ErrorMessage, command
        
def stop():
    command = 'stop'
    suction_off=rospy.ServiceProxy('/robot1_IOSignal',robot_IOSignal)
    
    if haverobot:
        suction_off(2,0)
    else:
        print '[Suction],', ErrorMessage, command

def pump_start():
    command = 'pump_start'
    pump_on=rospy.ServiceProxy('/robot1_IOSignal',robot_IOSignal)
    
    if haverobot: 
        pump_on(3,1)
    else:
        print '[Suction],', ErrorMessage, command
        
def pump_stop():
    command = 'pump_stop'
    pump_off=rospy.ServiceProxy('/robot1_IOSignal',robot_IOSignal)
    
    if haverobot:
        pump_off(3,0)
    else:
        print '[Suction],', ErrorMessage, command

if __name__=='__main__':
    rospy.init_node("suction_testing")
    

