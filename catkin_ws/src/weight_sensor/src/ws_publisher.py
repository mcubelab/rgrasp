#!/usr/bin/env python

import serial, rospy, time, sys, platform, numpy
from std_msgs.msg import Float64
from ast import literal_eval
from serial.tools.list_ports_posix import comports

def get_usbs():
    '''Query the machine for the weight sensor style usbs plugged in.
    Each weight sensor uses a set of four consecutive usbs. We return
    a list of the first in each set
    @return starting_ports The first usb for each sensor'''
    if platform.linux_distribution()[1] == '14.04':
        iterator = numpy.array(comports())[:, 1]
    else:
        iterator = [i.name for i in comports()]
    usbstring = 'ttyUSB'
    port_names = [i for i in iterator if i.startswith(usbstring)]
    port_numbers = sorted([int(p.lstrip(usbstring)) for p in port_names])
    starting_ports = [f for f in port_numbers if f % 4 == 0]
    return starting_ports

def sensor_ordering(usbs):
    '''Get the USB ordering of the sensors to match the publisher
    @param usbs Starting usbs for each sensor box
    @return ordering List of ports in proper order'''
    num_sensors = len(usbs)*4

    # Large bins, small bins, cardboard bins
    port_ordering = ['F172422208', 'F171721760', 'DIDIDIDIDWW']
    if len(usbs) != len(port_ordering):
        raise ValueError('Improper number of port IDs. Did not match sensor count')

    ordering = [0]*num_sensors
    for i in xrange(len(usbs)):
        # Sometimes when we read the sensor it does not not output the id
        # And instead prints garbage (this could be because they arent closed
        # properly?) So we loop till we get a proper id
        needID = True
        while needID:
            ser = serial.Serial(port='/dev/ttyUSB{}'.format(usbs[i]), timeout=0.5)
            ser.write('ID\r')
            ser.flush()
            time.sleep(0.05)
            out_t = ser.read(13)
            port_id = out_t[0:-2]
            if port_id in port_ordering: 
                id_label = 4*port_ordering.index(port_id)
                ordering[id_label:id_label+4] = range(usbs[i], usbs[i]+4)
                needID = False
            ser.close()

    return ordering


def talker_weights():
    '''Publish the measured weight according to the number of sensors in use
    (can be updated in the 'num_sensors' variable below) and if output desired
    is continous or not (can be updated in the 'continous' variable below).'''
    sensor_usbs = get_usbs()
    num_sensors = len(sensor_usbs)*4
    num_publish = num_sensors / 2
    continous = False

    # Start up the ROS publishers
    pub = []
    for i in xrange(num_publish):
        pub.append(rospy.Publisher('ws_stream{}'.format(i), Float64, queue_size=1))
    rospy.init_node('ws_talker', anonymous=True)
    rate = rospy.Rate(39)      # publishing (hz) - 40 highest

    # Based on the order of the USBs (set above), append the sensors
    # to line up with their publisher later on
    ser = []
    ordering = sensor_ordering(sensor_usbs)
    for i in ordering:
        ser.append(serial.Serial(port='/dev/ttyUSB{}'.format(i), timeout=None))

    # Set the mode of the sensor, either to continuously
    # Read or discretely poll
    if continous:
        mode_str = 'WC\r'
    else:
        mode_str = 'W\r'

    for i in xrange(num_sensors):
        (ser[i]).write(mode_str)
        (ser[i]).flush()
        (ser[i]).flushInput()
        (ser[i]).flushOutput()
        time.sleep(0.1)

    output = [0.0]*num_sensors
    output_str = ['']*num_sensors
    output_str_hold = ['']*num_sensors
    while not rospy.is_shutdown():
        if continous:
            for x in xrange(num_sensors):
                out_t = output_str_hold[x]
                bytesToRead = (ser[x]).inWaiting()
                #XXX The reason why continous doesnt work sometimes is the bytes
                # to read is 0. if you force it higher it hangs 
                out_t += (ser[x]).read(bytesToRead)
                if out_t.endswith(('\n', '\r', ' ')):
                    output_str[x] = out_t.split()[-1]
                    output_str_hold[x] = ''
                elif len(out_t.split()) >= 2:
                    # Otherwise we need to read the previous number
                    # If there is no previous number, keep the existing
                    output_str[x] = out_t.split()[-2]
                    output_str_hold[x] = out_t
        else:
            for x in xrange(num_sensors):
                (ser[x]).write(mode_str)
                (ser[x]).flush()
            time.sleep(0.05)             # 0.02 is highest
            output_str = ['']*num_sensors
            for y in xrange(num_sensors):
                output_str[y] = (ser[y]).read(14)

        # Attempt to convert the output of the weight sensor
        # Since we value error on an empty string,
        scale_factor = 453.592
        threshold = 14000
        for i in xrange(len(output)):
            try:
                tmp = float(output_str[i])*scale_factor
                if tmp < threshold:
                    output[i] = tmp
            except ValueError:
                pass

        # Print sensor output
        log_output = ''
        for i in xrange(num_sensors):
            log_output += '%6.2f ' % output[i]
        rospy.loginfo(log_output)

        # One publisher publishes the sum of two sensors
        weight_meas = [i+j for i, j in zip(output[::2], output[1::2])]
        # Publish the data
        ws_value = [Float64()]*num_publish
        for k in xrange(num_publish):
            (ws_value[k]).data = weight_meas[k]
            (pub[k]).publish(ws_value[k])

        rate.sleep()

    for s in ser:
        s.close()

def talker_contacts():
    '''Publish the measured weights and compare them to a threshold.'''
    #pub = rospy.Publisher('ws_stream', String, queue_size=10)
    pub = rospy.Publisher('ws_stream', Float64, queue_size=1)
    rospy.init_node('ws_talker', anonymous=True)
    rate = rospy.Rate(20)      # publishing (hz)
    #node publishes to ws_stream topic using message float64
    #node name = ws_talker

    ser_0 = serial.Serial(port='/dev/ttyUSB0', timeout=None)
    ser_1 = serial.Serial(port='/dev/ttyUSB1', timeout=None)

    if not ser_0.isOpen():
        ser_0.open()
    mode_str = 'W\r' # 'O0W1\r'

    while not rospy.is_shutdown(): #check if should exit: contrl c
        # read the port
        ser_0.write(mode_str)
        ser_0.flush()
        ser_1.write(mode_str)
        ser_1.flush()
        time.sleep(0.05)
        out_0 = ''
        out_1 = ''
        while ser_0.inWaiting() > 0:
            out_0 += ser_0.read(1)
            out_1 += ser_1.read(1)

        #~ print repr(out_0)
        #~ print repr(out_1)

        #~ map(float, temp.strip().split('\r\n'))

        # the following computations are in mlPounds
        with open('/tmp/zeros.txt', 'r') as f:
            for line in f:
                zeros = literal_eval(line)
        sensor_meas = (float(out_1)-zeros[1]+float(out_0)-zeros[0])

        push_force = sensor_meas*453.592

        threshhold = 20
        if push_force > threshhold:
            inContact = 1
        else:
            inContact = 0
        # publish the output of the port to the network
        #ws_value = '[WS] In contact: %d' % inContact
        ws_value = Float64()
        ws_value.data = inContact
        #rospy.loginfo(ws_value)
        pub.publish(ws_value)
        rate.sleep()

def main(withContact=False):
    '''Calibrate the sensors then report contact.
    @param withContact (bool) True to use as contact sensor
            and False to use as weight sensor'''
    #~ ws_prob.calibrate_zero()

    if withContact:
        try:
            talker_contacts()
        except rospy.ROSInterruptException:
            pass
    else:
        try:
            talker_weights()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    sys.exit(main(False))
    #sys.exit(main(sys.argv[1]))
