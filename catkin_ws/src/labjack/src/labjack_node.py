#!/usr/bin/env python

import u3
import numpy as np
import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty,EmptyResponse

d = u3.U3()
l = []
l2 = []
original_d = 0
buffer_d = 0
buffer_original_d = 0
def labjack_loop():
    global original_d, buffer_original_d
    pub = rospy.Publisher('flow_level', Float64, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        ainValue2 = d.getAIN(0)
        l.append(ainValue2)
        l2.append(ainValue2)
        if len(l) > 100:
            buffer_d = np.mean(l)
            pub.publish(buffer_d - original_d)
            #print '%.2f' % np.mean(l)
            l.pop(0)
            
        buffer_original_d = np.mean(l2)
        if len(l2) > 150:
            l2.pop(0)
        rate.sleep()
    d.close()

def calibrate_original_d(req):
    global original_d, buffer_original_d
    original_d = buffer_original_d
    return EmptyResponse()
    
def pre_calibrate_original_d(req):
    global l2
    l2=[]
    return EmptyResponse()

if __name__=='__main__':
    rospy.init_node('labjack_node')
    s = rospy.Service('flow_sensor_calibration', Empty, calibrate_original_d)
    s = rospy.Service('flow_sensor_empty_buffer', Empty, pre_calibrate_original_d)
    try:
        labjack_loop()
    except rospy.ROSInterruptException:
        d.close()
        pass
