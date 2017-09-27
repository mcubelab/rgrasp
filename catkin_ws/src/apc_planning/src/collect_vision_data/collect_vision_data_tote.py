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

limits = [-0.127,0.127, -0.236, 0.236, -0.20, -0.20]  #[xmin, xmax, ymin, ymax, zmin, zmax]
nseg = [3, 5, 1]
globalacc = 1             # some big number means no limit, in m/s^2

obj_id = "test_tote"
base_dir = "/home/mcube/apcdata/princeton_data/raw"
#setcart 1524.4 9.91 956.06 0.4312 -0.537 0.559 -0.4617
#setjoint 3.6 -0.17 16.92 -127.29 8.27 38.13

setTool(0,37,550,1,0,0,0)
rospy.init_node('collect_training_data')
setCartRos = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getCartRos = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setZone = rospy.ServiceProxy('/robot1_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot1_SetAcc', robot_SetAcc)
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

setZone(0)
setSpeed(500, 200)
#setSpeed(100, 20)
setAcc(acc=globalacc, deacc=globalacc)
setJoint(0,17.59,10.43,0,61.98,90.01) #tote



br = tf.TransformBroadcaster()
rospy.sleep(0.5)

center_pose = getCart()
tip_hand_transform = [0,0,0.55,0,0,0]

i = 0

xyzlist = []
for x in np.linspace(limits[0],limits[1], nseg[0]):
    for y in np.linspace(limits[2],limits[3], nseg[1]):
        for z in np.linspace(limits[4],limits[5], nseg[2]):
            xyzlist.append((x,y,z))
            i += 1


i = 0
first_frame_id = 0

import os
import errno
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
            
#make_sure_path_exists('%s/tmp/' % (base_dir))

for xyz in xyzlist:
        
    (x,y,z) = xyz
    
    setCart((np.array(center_pose[0:3]) + np.array(xyz)).tolist() , center_pose[3:7])
    rospy.sleep(1)
    # save one image
    det = detectObjects()
    
    camera_map_pose = poseTransform([0,0.037,0.192,0,0,0,1], '/link_6', '/map', listener)
    
    
    print 'camera_map_pose', camera_map_pose
    pubFrame(br, pose=camera_map_pose, frame_id='camera', parent_frame_id='/map', npub=1)
    
    # filename = '%s/tmp/frame-%06d.pose.txt'% (base_dir,det.frame_idx)
    # if i == 0:
        # first_frame_id = det.frame_idx
    # f = open(filename,'w')
    # m = matrix_from_xyzquat(camera_map_pose)
    # f.write('%f %f %f %f\n' % tuple(m[0])) 
    # f.write('%f %f %f %f\n' % tuple(m[1])) 
    # f.write('%f %f %f %f\n' % tuple(m[2])) 
    # f.write('%f %f %f %f\n' % tuple(m[3])) 
    # f.close() 
    pause()
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
