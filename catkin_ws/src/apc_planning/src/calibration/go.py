#!/usr/bin/env python

import rospy
import sys
from robot_comm.srv import robot_GetCartesian, robot_SetCartesian
# read cartesian

getCartRos = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setCartRos = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)

c = getCartRos()

dx,dy,dz = 0,0,0
if len(sys.argv) == 4:
    dx,dy,dz = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    
if len(sys.argv) == 2:
    dz = float(sys.argv[1])

setCartRos(c.x+dx, c.y+dy, c.z+dz, c.q0, c.qx, c.qy, c.qz)

# add cartesian 
