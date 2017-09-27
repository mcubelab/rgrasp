#!/usr/bin/env python

# this is to find out the transform between the vicon frame and robot frame

import tf.transformations as tfm
import numpy as np
from ik.roshelper import *
from ik.helper import *
import tf
import rospy
from tf.broadcaster import TransformBroadcaster
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *

from ik.roshelper import pose2list
from visualization_msgs.msg import Marker

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import tf.transformations as tfm
from ik.ik import setSpeed
from pr_apriltags.msg import AprilTagDetections

#limits = [0.32, 0.38, -0.2, +0.1, 0.27, 0.27]  #[xmin, xmax, ymin, ymax, zmin, zmax]
limits = [-0.1, 0.1, -0.39, -0.45, 0.55, 0.85]  #[xmin, xmax, ymin, ymax, zmin, zmax]
nseg = [3, 3, 3]
nrotate = 1
ori = [0, 0.7071, 0.7071, 0]
globalacc = 1             # some big number means no limit, in m/s^2

rospy.init_node('apriltag_vs_robot')
setCartRos = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getJoint = rospy.ServiceProxy('/robot1_GetJoints', robot_GetJoints)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setZone = rospy.ServiceProxy('/robot1_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot1_SetAcc', robot_SetAcc)
listener = tf.TransformListener()

def xyztolist(pos):
    return [pos.x, pos.y, pos.z]

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)
    
    
def findtagid(detections, tag_obj_id):
    for d in detections:
        if d.id == tag_obj_id:
            return d.pose
    return None

def norm(vect):
    vect = np.array(vect)
    return np.sqrt(np.dot(vect, vect))


br = tf.TransformBroadcaster()
rospy.sleep(0.1)

setZone(0)

xs = []
ys = []
zs = []
xt = []
yt = []
zt = []
setSpeed(100, 60)
setAcc(acc=globalacc, deacc=globalacc)

cam_poses = []

tt = 0
for x in np.linspace(limits[0],limits[1], nseg[0]):
    for y in np.linspace(limits[2],limits[3], nseg[1]):
        for z in np.linspace(limits[4],limits[5], nseg[2]):
            setCart([x,y,z], ori)
            print x,y,z
            # get the apriltag pose from pr_apriltag
            
            apriltag_topic = '/pr_apriltags/detections' 
            #print 'wait for detection at', apriltag_topic
            
            rospy.sleep(2)
            tag_camera_transform = None
            for i in xrange(5):
                tagdetect = ROS_Wait_For_Msg(apriltag_topic, AprilTagDetections).getmsg() 
                tagpose = findtagid(tagdetect.detections, 0)
                if tagpose is not None:
                    tag_camera_transform = pose2list(tagpose)
                    break
                rospy.sleep(0.02)
                
            if tag_camera_transform is None:
                print 'No tag detect'
                continue
            
            # get the link6 from robot
            (apriltrans,aprilrot) = lookupTransform('/map','/link6_apriltag', listener)
            appose_robot = list(apriltrans) + list(aprilrot)
            
            cam_pose = xyzquat_from_matrix( np.dot( matrix_from_xyzquat(appose_robot) , np.linalg.inv(matrix_from_xyzquat(tag_camera_transform))) )
            # appose_robot * inv(tagpose)
                
            xs = xs + [cam_pose[0]]
            ys = ys + [cam_pose[1]]
            zs = zs + [cam_pose[2]]
            
            cam_poses.append(cam_pose)
            #print cam_pose
            print tag_camera_transform
            pubFrame(br, cam_pose, '/new_tmp_realsense%d' % tt, '/map', 5)
            tt += 1
            
avg_campose = np.average(cam_poses, axis = 0)
std_campose = np.std(cam_poses, axis = 0)

print 'avg_campose:', " ".join('%.8e' % x for x in (avg_campose.tolist()))
print 'std_campose:', " ".join('%.8e' % x for x in (std_campose.tolist()))

# print 'data length', len(xs)
plt.scatter(xs, ys, c='r', marker='o')
# 
plt.show()
plt.scatter(ys, zs, c='b', marker='o')
plt.show()
# 
# viconpts = np.vstack([xs, ys, zs]).T
# robotpts = np.vstack([xt, yt, zt]).T
# 
# (R,t,rmse) = rigid_transform_3D(viconpts, robotpts)  # then you'll get vicon frame wrt robot frame
# 
# Rh = tfm.identity_matrix()
# Rh[np.ix_([0,1,2],[0,1,2])] = R
# quat = tfm.quaternion_from_matrix(Rh)
# 
# print 'vicon_T_robot:', " ".join('%.8e' % x for x in (t.tolist() + quat.tolist()))
# print 'rmse:', rmse
