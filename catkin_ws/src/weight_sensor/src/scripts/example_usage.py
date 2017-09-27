#!/usr/bin/env python
'''Example Script for the Weight Sensor, evaluates in simulation and real'''
import rospy, numpy, sys, time
sys.path.append('../')
from ws_prob import WeightSensor

def executeTest(W, real=True):
    '''Execute a test of the weight sensor by calibrating, asking the user
    to remove an object, and guessing what was removed.
    @param obj_list (list of strings) List of objects in the bin
    @param real (bool) Use real weight sensor or simulated'''
    #rospy.init_node('ws_prob')
    sim_weight = 0.5
    objList = ["Expo_Eraser", "Duct_Tape", "No Item"]
    b = 0

    # Calibrate Sensors
    W.calibrateWeights(withSensor=real)
    raw_input('Press any key when you have removed an object.')

    # Measure results (i.e not calibrating)
    if not real:
        try:
            sim_weight = float(raw_input('Enter fake resultant weight: '))
        except ValueError:
            print 'Entered invalid input, defaulting to {}'.format(sim_weight)
    ws_data = W.readWeightSensor(["Expo_Eraser", "Duct_Tape"], 
                       withSensor=real, binNum=0, givenWeights=sim_weight)

    print '[WS] Probabilities are: {}'.format(ws_data['probs'])
    print '[WS] Weights are: {}\n'.format(ws_data['weights'])
    max_num = numpy.argmax(ws_data['probs'])
    print '[WS] You probably removed: {}'.format(objList[max_num])

if __name__ == '__main__':
    rospy.init_node('weight_sensor_rosout_listener', anonymous=True)
    W = WeightSensor()
    time.sleep(1)
    print '[WS] Testing with real sensors...\n'
    executeTest(W, real=True)
    #print '[WS] Testing with fake sensors...\n'
    #executeTest(W, real=False)
