#!/usr/bin/env python
'''Example Script for the Weight Sensor, evaluates in simulation and real'''
import rospy, sys
sys.path.append('../')
from ws_prob import WeightSensor

def binTest(W):
    '''Script to test measured weight resuls from each bin. First, calibrate
    each bin. Then allow the user to move items around and get the weight
    difference'''

    rospy.init_node('ws_prob')
    #objList = ["Expo_Eraser", "Duct_Tape", "No Item"]
    binCount = 8

    W.calibrateWeights(withSensor=real)
    raw_input('Press any key when you have finished moving objects. ')

    for b in xrange(binCount):    
        W.readWeightSensor(["Expo_Eraser", "Duct_Tape"], binNum=b)

if __name__ == '__main__':
    W = WeightSensor()
    binTest(W)
