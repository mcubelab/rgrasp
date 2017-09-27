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

limits = [-40,10, -40, 40, 0, 0]  #[up, down, left, right, zmin, zmax]
limits = [-30,10, -30, 30, 0, 0]  #[up, down, left, right, zmin, zmax] bin 04
limits = [-20,10, -25, 25, 0, 0]  #[up, down, left, right, zmin, zmax] bin 02
limits = [-20,0, -25, 25, 0, 0]  #[up, down, left, right, zmin, zmax] bin 11
nseg = [3, 5, 1]
globalacc = 1             # some big number means no limit, in m/s^2

obj_id = "womens_knit_gloves"
base_dir = "/home/mcube/apcdata/princeton_data/raw"
#setcart 1524.4 9.91 956.06 0.4312 -0.537 0.559 -0.4617
#setjoint 3.6 -0.17 16.92 -127.29 8.27 38.13

rospy.init_node('collect_training_data')
setCartRos = rospy.ServiceProxy('/robot1_SetCartesianJ', robot_SetCartesian)
getCartRos = rospy.ServiceProxy('/robot1_GetCartesian', robot_GetCartesian)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setZone = rospy.ServiceProxy('/robot1_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot1_SetAcc', robot_SetAcc)
detectObjects = rospy.ServiceProxy('/apc_vision', DetectObjects)
setTool = rospy.ServiceProxy('/robot1_SetTool', robot_SetTool)
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

setTool(0,-37,550,0,0,0,-1)
setZone(0)
setSpeed(500, 200)
setAcc(acc=globalacc, deacc=globalacc)
setJoint(0.02,1.66,29.01,0.04,-30.68,+90.04) #bin4
#setJoint(-19.16,5.31,25.09,-34.48,-35.44,-60.77)  #bin5
#setJoint(-19.16,1.98,4.38,-72.31,-20.15,-18.76)  #bin2
#setJoint(-19.16,34.59,40.1,-19.81,-75.56,-84.87)  #bin11
# j1: -19.16
# j2: 34.59
# j3: 40.1
# j4: -19.81
# j5: -75.56
# j6: -84.87



br = tf.TransformBroadcaster()
rospy.sleep(0.5)

center_pose = getCart()
tip_hand_transform = [0,0,0.55,0,0,0]

# j1: -19.16
# j2: 5.31
# j3: 25.09
# j4: -34.48
# j5: -35.44
# j6: -60.77


i = 0

xyzlist = []
for x in np.linspace(limits[0],limits[1], nseg[0]):
    for y in np.linspace(limits[2],limits[3], nseg[1]):
        for z in np.linspace(limits[4],limits[5], nseg[2]):
            xyzlist.append((x,y,z))


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
    if i % nseg[1] == 0:
        setJoint(0.02,1.66,29.01,0.04,-30.68,+90.04) #bin4
        
    (x,y,z) = xyz
    #setJoint(-19.16,5.31,25.09,-34.48,-35.44,-60.77)  #bin5
    #setJoint(-19.16,1.98,4.38,-72.31,-20.15,-18.76)  #bin2
    #setJoint(-19.16,34.59,40.1,-19.81,-75.56,-84.87)  #bin11
    
    #setJoint(0.03,-10.27,39.9,0.05,-29.63,-90.05)
    rotate = np.dot( matrix_from_xyzrpy([0,0,0,0,np.deg2rad(y),0]), matrix_from_xyzrpy([0,0,0,np.deg2rad(x),0,0]) )
    newpose = xyzquat_from_matrix( np.dot(matrix_from_xyzquat(center_pose), rotate))
    
    
    #planner = IK(target_tip_pos = newpose[0:3], target_tip_ori = newpose[3:7], 
    #             tip_hand_transform=tip_hand_transform) # straightness: [0-1] at what point it should exactly follow the target orientation
    #plan = planner.plan()
    # if plan.success():
        # plan.visualize()
        # pause()
        # plan.execute()
    # else:
        # print 'fail'
    
    #pause()
    setCart(newpose[0:3], newpose[3:7])
    print 'before sleep'
    rospy.sleep(0.2)
    print 'after sleep'
    # save one image
    det = detectObjects()
    print 'after detect'
    
    camera_map_pose = poseTransform([0,-0.037,0.192,0,0,-1,0], '/link_6', '/map', listener)
    
    
    #print 'camera_map_pose', camera_map_pose
    pubFrame(br, pose=camera_map_pose, frame_id='camera', parent_frame_id='/map', npub=1)
    
    filename = '%s/tmp/frame-%06d.pose.txt'% (base_dir,det.frame_idx)
    if i == 0:
        first_frame_id = det.frame_idx
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
