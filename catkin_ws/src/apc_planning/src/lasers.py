#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 17 15:21:37 2017

@author: mcube
"""

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from realsense_camera.srv import start_lasers

ErrorMessage = 'Spatula not connected, skipping command: '
haveraspberry = rospy.get_param('/use_raspberry', True)


def start(bin_id):
    lasers = rospy.ServiceProxy('/arc_1/realsense_camera/start_lasers', start_lasers)
    
    if haveraspberry:
        lasers(bin_id)
        print '[Lasers] Start lasers in bin: ', bin_id
    else:
        print '[Lasers] Raspberry no connected' 

       
def stop(bin_id):  
    lasers = rospy.ServiceProxy('/arc_1/realsense_camera/stop_lasers', start_lasers)
    
    if haveraspberry:
        lasers(bin_id)
        print '[Lasers] Stop lasers in bin: ', bin_id
    else:
        print '[Lasers] Raspberry no connected' 
        
if __name__=='__main__':
  rospy.init_node("lasers_testing")
