#!/usr/bin/env python

from sensor_msgs.msg import LaserScan
from tf import *
import ik.roshelper
from ik.helper import *
import rospy
import os
import numpy as np


rospy.init_node("laser_scan_test")

d = []
angle_min = 0
angle_max = 0
angle_increment = 0

def callback(data):
    print len(d)
    d.append(data.ranges)
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
    
    if len(d) in [1,10,100,1000]:
        ad = np.average(np.array(d), axis = 0)
        xys = []
        for i, r in enumerate(list(ad)):
            x,y = r * np.cos(angle_min + angle_increment * i) , r * np.sin(angle_min + angle_increment * i) 
            if  0 < y < 0.08 and x > 0.1 and x < 2:
                print x,y
                xys.append((x,y))
            
        xs, ys = zip(*xys)

        ax = np.average(xs)
        print 'average x', ax
        
        import matplotlib.pyplot as plt
        plt.plot(xs, ys, 'ro')
        plt.axis('equal')
        plt.show()
        



rospy.Subscriber("/scan", LaserScan, callback)

#for i in xrange(100):
rospy.spin()
    



