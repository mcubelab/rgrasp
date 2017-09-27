#!/usr/bin/env python

import rospy
import sys
from robot_comm.srv import robot_GetJoints, robot_SetJoints
# read joint

getJointRos = rospy.ServiceProxy('/robot1_GetJoints', robot_GetJoints)
setJointRos = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)

js = getJointRos()

dj = [0.0]*6
if len(sys.argv) == 7:
    dj = [float(j) for j in sys.argv[1:7]]
 
elif len(sys.argv) == 2:
    dj[5] = float(sys.argv[1])

else:
    print 'gojoint dj1 dj2 dj3 dj4 dj5 dj6'
    print 'gojoint dj6'



setJointRos(js.j1+dj[0], js.j2+dj[1], js.j3+dj[2], js.j4+dj[3], js.j5+dj[4], js.j6+dj[5])

# add cartesian 
