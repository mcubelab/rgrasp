#!/usr/bin/env python

import sys
import serial
import time

def main(withStream):
    ser = serial.Serial('/dev/ttyUSB0')
    # ser = serial.Serial(port='/dev/ttyUSB0', baudrate=19200)

    if not ser.isOpen():
        ser.open()

    if withStream:
        mode_str = 'O0W0'
    else:
        mode_str = 'O0W1'
    mode_str = 'W\r'
    ser.write(mode_str)
    ser.flush()
    time.sleep(0.05)
    print ser.read(1)
    ser.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
