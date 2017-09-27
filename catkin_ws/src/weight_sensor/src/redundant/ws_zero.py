#!/usr/bin/env python

import time
import serial
import rospy
import json

usb_list = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']
num_samples = 9

if __name__=="__main__":
    ser =[]
    for i in range(0,4):
        ser.append(serial.Serial(port=usb_list[i]))
        if ser[i].isOpen() == False:
            ser[i].open()
        print 'Is serial port %d open? %s' % (i,ser[i].isOpen())
    
    mean = [0.0,0.0,0.0,0.0]

    for i in range(0,num_samples):
        for j in range(0,4):
            ser[j].write('W\r') #command to read the weight
            ser[j].flush()
            out = ''
            time.sleep(0.1)
            while ser[j].inWaiting() > 0:
                out +=ser[j].read(1)
            mean[j] = float(out)+mean[j]
    
    zeros = [x/num_samples for x in mean]
    with open('/tmp/zeros.txt', 'w') as outfile:
        json.dump(zeros, outfile)
