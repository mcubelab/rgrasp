#!/usr/bin/env python

import tf.transformations as tfm
import numpy as np
from ik.roshelper import *
from ik.helper import *
from ik.ik import IK
import tf
import rospy
from tf.broadcaster import TransformBroadcaster
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *

from apc_vision.srv import *


from ik.roshelper import pose2list
from visualization_msgs.msg import Marker

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import tf.transformations as tfm
from ik.ik import setSpeed

# points of uniform sampling of z>0 hemisphere
points = [[-0.961938, 0.000000, 0.273267],[-0.951057, -0.262866, 0.162460],[-0.951057, 0.262866, 0.162460],[-0.850651, 0.000000, 0.525731],[-0.862668, -0.259892, 0.433889],[-0.862668, 0.259892, 0.433889],[-0.809017, -0.500000, 0.309017],[-0.809017, 0.500000, 0.309017],[-0.702046, -0.160622, 0.693780],[-0.702046, 0.160622, 0.693780],[-0.693780, -0.702046, 0.160622],[-0.693780, 0.702046, 0.160622],[-0.688191, -0.425325, 0.587785],[-0.688191, 0.425325, 0.587785],[-0.587785, -0.688191, 0.425325],[-0.587785, 0.688191, 0.425325],[-0.500000, -0.309017, 0.809017],[-0.525731, 0.000000, 0.850651],[-0.500000, 0.309017, 0.809017],[-0.433889, -0.862668, 0.259892],[-0.433889, 0.862668, 0.259892],[-0.425325, -0.587785, 0.688191],[-0.425325, 0.587785, 0.688191],[-0.309017, -0.809017, 0.500000],[-0.309017, 0.809017, 0.500000],[-0.259892, -0.433889, 0.862668],[-0.262866, -0.162460, 0.951057],[-0.262866, 0.162460, 0.951057],[-0.259892, 0.433889, 0.862668],[-0.162460, -0.951057, 0.262866],[-0.160622, -0.693780, 0.702046],[-0.160622, 0.693780, 0.702046],[-0.162460, 0.951057, 0.262866],[0.000000, -0.850651, 0.525731],[0.000000, -0.525731, 0.850651],[0.000000, -0.273267, 0.961938],[0.000000, 0.000000, 1.000000],[0.000000, 0.273267, 0.961938],[0.000000, 0.525731, 0.850651],[0.000000, 0.850651, 0.525731],[0.162460, -0.951057, 0.262866],[0.160622, -0.693780, 0.702046],[0.160622, 0.693780, 0.702046],[0.162460, 0.951057, 0.262866],[0.259892, -0.433889, 0.862668],[0.262866, -0.162460, 0.951057],[0.262866, 0.162460, 0.951057],[0.259892, 0.433889, 0.862668],[0.309017, -0.809017, 0.500000],[0.309017, 0.809017, 0.500000],[0.425325, -0.587785, 0.688191],[0.425325, 0.587785, 0.688191],[0.433889, -0.862668, 0.259892],[0.433889, 0.862668, 0.259892],[0.500000, -0.309017, 0.809017],[0.525731, 0.000000, 0.850651],[0.500000, 0.309017, 0.809017],[0.587785, -0.688191, 0.425325],[0.587785, 0.688191, 0.425325],[0.688191, -0.425325, 0.587785],[0.688191, 0.425325, 0.587785],[0.693780, -0.702046, 0.160622],[0.693780, 0.702046, 0.160622],[0.702046, -0.160622, 0.693780],[0.702046, 0.160622, 0.693780],[0.809017, -0.500000, 0.309017],[0.809017, 0.500000, 0.309017],[0.862668, -0.259892, 0.433889],[0.862668, 0.259892, 0.433889],[0.850651, 0.000000, 0.525731],[0.951057, -0.262866, 0.162460],[0.951057, 0.262866, 0.162460],[0.961938, 0.000000, 0.273267]]


limits = [0,0,0,0, -np.pi, +np.pi]  #[up, down, left, right, zmin, zmax] bin 11
limits = [0,0,0,0, 0,0]  #[up, down, left, right, zmin, zmax] bin 11
nseg = [3, 5, 1]
globalacc = 1             # some big number means no limit, in m/s^2

obj_id = "expo_dry_erase_board_eraser"
obj_id = "ontote_elmers_washable_no_run_school_glue"
base_dir = "/home/mcube/apcdata/princeton_data/raw"
#setcart 1524.4 9.91 956.06 0.4312 -0.537 0.559 -0.4617
#setjoint 3.6 -0.17 16.92 -127.29 8.27 38.13

rospy.init_node('collect_training_data')
setCartRos = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getCartRos = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
getJoint = rospy.ServiceProxy('/robot1_GetJoints', robot_GetJoints)
setZone = rospy.ServiceProxy('/robot1_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot1_SetAcc', robot_SetAcc)
setTool = rospy.ServiceProxy('/robot1_SetTool', robot_SetTool)
detectObjects = rospy.ServiceProxy('/apc_vision', DetectObjects)
listener = tf.TransformListener()


def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori[3:4] + ori[0:3] 
    print 'setCart', param
    #pause()
    setCartRos(*param)
    
def getCart():
    #pause()
    ret = getCartRos()
    print 'getCart', ret
    return [ret.x/1000, ret.y/1000, ret.z/1000, ret.qx, ret.qy, ret.qz, ret.q0]

# assuming tool is pointing at the object

setTool(0,37,500,1,0,0,0)
setZone(0)
setSpeed(20, 20)
setAcc(acc=globalacc, deacc=globalacc)
#setJoint(0.0,6.84,24.01,0,59.15,90.02) 
setJoint(0.0,28.56,-19.39,0,80.83,90) 
br = tf.TransformBroadcaster()
rospy.sleep(0.5)

center_pose = getCart()
tip_hand_transform = [0,0.037,0.55,0,0,0]

i = 0

xyzlist = []
for pt in points:
    if pt[2] < 0.7:
        continue
    x = np.arcsin((pt[0]**2+pt[1]**2)**0.5)
    y = np.arctan2(pt[1], pt[0])
    #y = 0
    for z in np.linspace(limits[4],limits[5], nseg[2], endpoint=False):
            
        xyzlist.append((x,y,z))
        i += 1

print xyzlist
xyzlist = [(1.4,1.57,0)]
pause()

i = 0

for xyz in xyzlist:
    # if i % nseg[2] == 0:
    #setJoint(0.0,6.84,24.01,0,59.15,90.02) 
    setJoint(0.0,28.56,-19.39,0,80.83,90) 
    
    
    (x,y,z) = xyz
    rotate = np.dot(matrix_from_xyzrpy([0,0,0,0,0,y]), np.dot( matrix_from_xyzrpy([0,0,0,0,x,0]), matrix_from_xyzrpy([0,0,0,0,0,z]) ) )
    #rotatematrix_from_xyzrpy([0,0,0,x,y,z])
    newpose = xyzquat_from_matrix( np.dot(matrix_from_xyzquat(center_pose), rotate))
    
    #pause()
    setCart(newpose[0:3], newpose[3:7])
    
    # avoid twisting arm
    # if i % nseg[2] == 0 or:
        # joints = getJoint()
        # setJoint(joints.j1,joints.j2,joints.j3,joints.j4,joints.j5,0)
        # setCart(newpose[0:3], newpose[3:7])
        
    rospy.sleep(0.5)
    # save one image
    det = detectObjects()
    
    camera_map_pose = poseTransform([0,0.037,0.192,0,0,0,1], '/link_6', '/map', listener)
    
    
    print 'camera_map_pose', camera_map_pose
    pubFrame(br, pose=camera_map_pose, frame_id='camera', parent_frame_id='/map', npub=1)
    
    filename = '%s/tmp/frame-%06d.pose.txt'% (base_dir,det.frame_idx)
    
    f = open(filename,'w')
    m = matrix_from_xyzquat(camera_map_pose)
    f.write('%f %f %f %f\n' % tuple(m[0])) 
    f.write('%f %f %f %f\n' % tuple(m[1])) 
    f.write('%f %f %f %f\n' % tuple(m[2])) 
    f.write('%f %f %f %f\n' % tuple(m[3])) 
    f.close() 
    
    i += 1

import shutil
import os

dirpath = "%s/%s" % (base_dir,obj_id)
def get_capture_id(dirpath_):
    i = 0
    while True:
        if not os.path.isdir("%s/%06d" % (dirpath_, i)):
            return i
        i += 1
      
        
shutil.copytree("%s/tmp" % base_dir, "%s/%06d" % (dirpath, get_capture_id(dirpath) ))
shutil.rmtree("%s/tmp" % base_dir)
