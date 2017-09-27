#!/usr/bin/env python

import rospy
import suction
import suction_calibration_flow
import gelsight
import spatula
import scorpion
import thread, time
from ik.ik import GraspingGuard
import gripper

def input_thread(L):
    derp = raw_input()
    L.append(derp)
    
    
    
def read_flow_sensor():
    L = []
    thread.start_new_thread(input_thread, (L,))
    while 1:
        suction_calibration_flow.read_flow_level()
        time.sleep(.1)
        if L:
            print L[0]
            del L[0]
            break
            
def test_guards():

    gelsight.lights_on()
    grasp_guard = GraspingGuard()
    grasp_guard.prep()
    L = []
    thread.start_new_thread(input_thread, (L,))

    while 1:
        rospy.sleep(0.1)
        print grasp_guard.test()
        
        if L:
            print L[0]
            del L[0]
            break

    gelsight.lights_off()

def test_suction():
#***************SUCTION***************
    print "TESTING SUCTION"
    raw_input("Press Enter to continue...")
    
#pump on
    print "Turning on pump"
    raw_input("Press Enter to continue...")
    suction.pump_start()
    
#suction on
    print "Turning on suction"
    raw_input("Press Enter to continue...")
    suction.start()

#check flow sensor
    print "check flow sensor"
    raw_input("Press Enter to continue...")
    read_flow_sensor()

#suction off
    print "Turning off suction"
    raw_input("Press Enter to continue...")
    suction.stop()

#pump off
    print "Turning off pump"
    raw_input("Press Enter to continue...")
    suction.pump_stop()

def test_gripper():
#***************GRIPPER********************
    print "TESTING GRIPPER, closing gripper"
    raw_input("Press Enter to continue...")
    gripper.close()
    
#scorpion fwd
    print "Scorpion Fwd"
    raw_input("Press Enter to continue...")
    scorpion.fwd()
    
#scorpion back
    print "Scorpion Back"
    raw_input("Press Enter to continue...")
    scorpion.back()
#finger closed
    print "Spatula Close"
    raw_input("Press Enter to continue...")
    gripper.open()
    spatula.close()

#finger open
    print "Spatula Open"
    raw_input("Press Enter to continue...")
    spatula.open()
    
    
#gelsight guard
    print "Gelsight Reading"
    gelsight.lights_on()
    raw_input("Press Enter to continue...")
    test_guards()
    gelsight.lights_off()
    raw_input("Press Enter to continue...")

if __name__=="__main__":
    rospy.init_node('test_hardware')

    #test_suction()
    test_gripper()
    print "DONE - remember to test electromagnet and weight sensors"


