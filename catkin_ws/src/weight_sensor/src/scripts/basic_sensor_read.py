#By David Durst
#same code will work for DI-100, DI-1000 or iLoad Series load cells
# 1. install VCP Drivers From (http://www.ftdichip.com/Drivers/VCP.htm)
# 2. Install PySerial (easy_install pyserial)
# 3. Identify the scale device (ls /dev/ |grep cu).
# You are looking for a device that starts with "cu.usbserial"
# 4. Update code to specify the exact device (Found on Line 4)
#!/usr/bin/env python
import time
import serial
# ser = serial.Serial(port='/dev/cu.usbserial-XXXXXX')
ser = serial.Serial(port='/dev/ttyUSB0')
if ser.isOpen() == False:
  ser.open()
print ser.isOpen()

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

print 'Mean is %f', mean/9

