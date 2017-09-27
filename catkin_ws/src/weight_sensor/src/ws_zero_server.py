#!/usr/bin/env python

import time
import serial
import rospy


def port_mean(port_str):
    ser = serial.Serial(port=port_str)
    if ser.isOpen() == False:
        ser.open()
    print 'Is serial port open? ', ser.isOpen()

    mean = 0.0

    for i in range(0,9):
        ser.write('W\r') #command to read the weight
        ser.flush()
        out = ''
        time.sleep(0.5)
        while ser.inWaiting() > 0:
            out +=ser.read(1)
        mean = float(out)+mean
        print out

    return mean/9
    

def handle_weight_sensor_zero(toteNum):
    print 'Zeroing weight sensors'

    if toteNum == 1:
        mean_1 = port_mean('/dev/ttyUSB0')
        mean_2 = port_mean('/dev/ttyUSB1')
    if toteNum == 2:
        mean_1 = port_mean('/dev/ttyUSB2')
        mean_2 = port_mean('/dev/ttyUSB3')
    
    return (mean_1,mean_2)

def weight_sensor_zero_server():
    rospy.init_node('weight_sensor_zero')
    s = rospy.Service('ws_zero', AddTwoInts, handle_weight_sensor_zero)
    print "Ready to zero."
    rospy.spin()

if __name__=="__main__":
    weight_sensor_zero_server()
