#!/usr/bin/env python

import serial
import time

def main():
    ser_0 = serial.Serial(port = '/dev/ttyUSB0', timeout = None)
    ser_0.write('ID\r')
    ser_0.flush()
    time.sleep(0.05)
    
    out_0 = ''
    while ser_0.inWaiting() > 0:
        out_0 += ser_0.read(1)
    print out_0[0:-2]
    print out_0[0:-2] == 'DIDIDIDIDWW'

if __name__ == '__main__':
    main()
