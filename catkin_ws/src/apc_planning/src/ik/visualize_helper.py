import os, errno
import json
import sys
import numpy as np
import tf
import rospy
import tf.transformations as tfm
import time
from visualization_msgs.msg import MarkerArray, Marker
from marker_helper import createCubeMarker2, createSphereMarker, createArrowMarker
from numpy import linalg as la
import traceback
from roshelper import lookupTransform, poseTransform, coordinateFrameTransformList, pubFrame
from helper import reference_frames, get_params_yaml, quat_from_matrix
import gripper
import yaml
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import Delaunay
from suction_projection import suction_projection_func
from roshelper import ROS_Wait_For_Msg
from sensor_msgs.msg import JointState
#from ik.marker_helper import createArrowMarker, createCubeMarker2, createDeleteAllMarker

#~copied from flush_grasping17.py
def get_flush_hand_frame(objInput, binId, listener, br):
    world_X, world_Y, world_Z, tote_X, tote_Y, tote_Z, tote_pose_pos = reference_frames(listener=listener, br=br)
    bin_center_pose = get_params_yaml('bin'+str(binId)+'_pose')
    bin_center_pos=np.array(bin_center_pose[0:3])
    bin_width=rospy.get_param("/tote/width")
    gelsight_fing_offset=0.030

    if  objInput[1]>bin_center_pos[1]:
        flush_mode='left'

        hand_Z=np.array(-world_Z)
        hand_X=np.array(world_Y)
        hand_Y=np.array(world_X)

        grasp_posY = bin_center_pos[1]+bin_width/2.0-objInput[4]/2.0-gelsight_fing_offset
        grasp_posX = objInput[0]
        grasp_posZ = objInput[2]-objInput[3]
        graspPos = np.array([grasp_posX, grasp_posY, grasp_posZ])

    else:
        flush_mode='right'

        hand_Z=np.array(-world_Z)
        hand_X=np.array(-world_Y)
        hand_Y=np.array(-world_X)

        grasp_posY = bin_center_pos[1]-bin_width/2.0+objInput[4]/2.0+gelsight_fing_offset
        grasp_posX = objInput[0]
        grasp_posZ = objInput[2]-objInput[3]
        graspPos = np.array([grasp_posX, grasp_posY, grasp_posZ])

    grasp_width = objInput[4]
    return graspPos, hand_X, hand_Y, hand_Z, grasp_width
    
#~copied from grasping17.py
def get_picking_params_from_12(objInput):  # copied from picking17
    #~define variables
    grasp_begin_pt=np.array(objInput[0:3])
    hand_Z=np.array(objInput[3:6])

    #~define grasp pos
    grasp_depth=np.array(objInput[6])
    graspPos=grasp_begin_pt + hand_Z*grasp_depth
    grasp_width=np.array(objInput[7])

    #~define hand frame
    hand_X=np.array(objInput[8:11])
    hand_Y=np.cross(hand_Z, hand_X)

    return graspPos, hand_X, hand_Y, hand_Z, grasp_width
    
def visualize_flush_proposals(proposal_viz_array_pub, proposals, binId, listener, br, is_selected = False):

    n = proposals.shape[0]
    if n>0 and proposals[0] is None:
        return

    color = (0, 1, 1, 1)  # rgba
    if is_selected:
        color = (1, 1, 0, 1)
    color_bar = (0.5, 0.5, 0.5, 0.5)  # rgba
    scale = (0.001,0.02,0.1)
    markers_msg = MarkerArray()
    #if not is_selected:  # don't delete other candidate
    #    markers_msg.markers.append(createDeleteAllMarker('pick_proposals'))
    subrate = 1
    if n > 5:
        subrate = n/5
    for i in xrange(0,n,subrate):
        pick_proposal = proposals[i,:]
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = get_flush_hand_frame(pick_proposal, binId=binId, listener=listener, br=br)
        scale_bar = (grasp_width,0.003,0.1)
        #import ipdb; ipdb.set_trace()
        rotmat = np.vstack((hand_X, hand_Y, hand_Z, np.zeros((1,3)))).T
        rotmat = np.vstack((rotmat, np.array([[0,0,0,1]])))
        quat = quat_from_matrix(rotmat)
        m1 = createCubeMarker2(rgba = color, scale = scale, offset = tuple(graspPos+hand_X*grasp_width/2), orientation= tuple(quat), marker_id = i*3, ns = 'flush_proposals')
        m2 = createCubeMarker2(rgba = color, scale = scale, offset = tuple(graspPos-hand_X*grasp_width/2), orientation= tuple(quat), marker_id = i*3+1, ns = 'flush_proposals')
        m3 = createCubeMarker2(rgba = color_bar, scale = scale_bar, offset = tuple(graspPos), orientation= tuple(quat), marker_id = i*3+2, ns = 'flush_proposals')

        markers_msg.markers.append(m1)
        markers_msg.markers.append(m2)
        markers_msg.markers.append(m3)
    
    proposal_viz_array_pub.publish(markers_msg)
    

def visualize_suction_points(proposal_viz_array_pub, all_suction_points, colorid):
    # self.all_suction_points  n*7 np.array
    # colorid = 'g', 'dg'
    n = all_suction_points.shape[0]
    colors = {'g':(0, 1, 0, 1) , 'dg':(0, 0.5, 0, 1)}
    rgba = colors[colorid]  # rgba
    scale = 0.03
    markers_msg = MarkerArray()
    if colorid == 'g':
        markers_msg.markers.append(createDeleteAllMarker('suction_points'))
    subrate = 1
    if n > 100:
        subrate = n/100
    for i in xrange(0,n,subrate):
        suction_point = all_suction_points[i,:]
        start = suction_point[0:3]
        direc = suction_point[3:6]
        end = start + direc/np.linalg.norm(direc) * scale
        m = createArrowMarker(start, end, rgba, marker_id = i, ns = 'suction_points' + colorid)
        markers_msg.markers.append(m)
    proposal_viz_array_pub.publish(markers_msg)
    #pauseFunc(True)

def visualize_suction_point(proposal_viz_array_pub, suction_point, colorid):
    # suction_point  len 7
    if suction_point is None:
        return
    colors = {'b':(0, 0, 1, 1) , 'db':(0, 0, 0.5, 1)}
    color = colors[colorid]  # rgba
    scale = 0.1
    markers_msg = MarkerArray()
    start = suction_point[0:3]
    direc = suction_point[3:6]
    end = start + direc/np.linalg.norm(direc) * scale
    m = createArrowMarker(start, end, color, marker_id = 10000, ns = 'suction_points'+ colorid)
    markers_msg.markers.append(m)
    proposal_viz_array_pub.publish(markers_msg)
    #pauseFunc(True)

def visualize_grasping_proposals(proposal_viz_array_pub, proposals, is_selected = False):

    # visualize all_grasp_proposals 2d list (n * 12)
    def get_picking_params_from_12(objInput):  # copied from picking17
        #~define variables
        grasp_begin_pt=np.array(objInput[0:3])
        hand_Z=np.array(objInput[3:6])

        #~define grasp pos
        grasp_depth=np.array(objInput[6])
        graspPos=grasp_begin_pt + hand_Z*grasp_depth
        grasp_width=np.array(objInput[7])

        #~define hand frame
        hand_X=np.array(objInput[8:11])
        hand_Y=np.cross(hand_Z, hand_X)

        return graspPos, hand_X, hand_Y, hand_Z, grasp_width

    n = proposals.shape[0]
    if n>0 and proposals[0] is None:
        return

    color = (0, 1, 1, 1)  # rgba
    if is_selected:
        color = (1, 1, 0, 1)
    color_bar = (0.5, 0.5, 0.5, 0.5)  # rgba
    scale = (0.001,0.02,0.1)
    markers_msg = MarkerArray()
    #if not is_selected:  # don't delete other candidate
    #    markers_msg.markers.append(createDeleteAllMarker('pick_proposals'))
    subrate = 1
    if n > 5:
        subrate = n/5
    for i in xrange(0,n,subrate):
        pick_proposal = proposals[i,:]
        graspPos, hand_X, hand_Y, hand_Z, grasp_width = get_picking_params_from_12(pick_proposal)
        scale_bar = (grasp_width,0.003,0.1)
        #import ipdb; ipdb.set_trace()
        rotmat = np.vstack((hand_X, hand_Y, hand_Z, np.zeros((1,3)))).T
        rotmat = np.vstack((rotmat, np.array([[0,0,0,1]])))
        quat = quat_from_matrix(rotmat)
        m1 = createCubeMarker2(rgba = color, scale = scale, offset = tuple(graspPos+hand_X*grasp_width/2), orientation= tuple(quat), marker_id = i*3, ns = 'pick_proposals')
        m2 = createCubeMarker2(rgba = color, scale = scale, offset = tuple(graspPos-hand_X*grasp_width/2), orientation= tuple(quat), marker_id = i*3+1, ns = 'pick_proposals')
        m3 = createCubeMarker2(rgba = color_bar, scale = scale_bar, offset = tuple(graspPos), orientation= tuple(quat), marker_id = i*3+2, ns = 'pick_proposals')

        markers_msg.markers.append(m1)
        markers_msg.markers.append(m2)
        markers_msg.markers.append(m3)
    proposal_viz_array_pub.publish(markers_msg)
    #pauseFunc(True)

