#!/usr/bin/python

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import random
from ik.helper import *
from ik.roshelper import *
import os
import json
import sys
import pdb
import numpy as np
import tf
import rospy
import tf.transformations as tfm
import time
from datetime import datetime
import case_generator
import copy

def plot_shape(l1, l2, l3, x, y, theta, ax, color):
    c = math.cos(theta)
    s = math.sin(theta)
    def rotate_x(X, Y):
        result = c*l1*X -s*l2*Y
        return result
    def rotate_y(X, Y):
        result = s*l1*X +c*l2*Y
        return result
    r = [-0.5,0.5]
    X, Y = np.meshgrid(r, r)
   # ax.plot_surface(rotate_x(X,Y)+x, rotate_y(X,Y)+y,l3, alpha=0.5, color = color)
    ax.plot_surface(rotate_x(X,Y)+x, rotate_y(X,Y)+y,0, alpha=0.5, color = color)
    
    ax.plot_surface(rotate_x(X,-0.5)+x,rotate_y(X,-0.5)+y,l3*(0.5+Y), alpha=0.5, color = color)
    ax.plot_surface(rotate_x(X,0.5)+x, rotate_y(X,0.5)+y,l3*(0.5+Y), alpha=0.5, color = color)
    ax.plot_surface(rotate_x(0.5,X)+x, rotate_y(0.5,X)+y,l3*(0.5+Y), alpha=0.5, color = color)
    ax.plot_surface(rotate_x(-0.5,X)+x, rotate_y(-0.5,X)+y,l3*(0.5+Y), alpha=0.5, color = color)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

def ccw(Ax, Ay, Bx, By, Cx, Cy):
    return (Cy-Ay)*(Bx-Ax) > (By-Ay)*(Cx-Ax)

def segment_intersection(Ax, Ay, Bx, By, Cx, Cy, Dx, Dy):
    return ccw(Ax, Ay,Cx, Cy,Dx, Dy) != ccw(Bx, By,Cx, Cy,Dx, Dy) and ccw(Ax, Ay,Bx, By,Cx, Cy) != ccw(Ax, Ay,Bx, By,Dx, Dy)

def rotation_x(theta, x, y):
    c = math.cos(theta)
    s = math.sin(theta)
    result = c*x -s*y
    return result

def rotation_y(theta, x, y):
    c = math.cos(theta)
    s = math.sin(theta)
    result = s*x +c*y
    return result

def read_objlist():
    jsonFilename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/object_data/allnewobjects.json'
    with open(jsonFilename) as data_file:
        ret = json.load(data_file) 
    return ret

def case_generator_tote(listener, br):
    
    # 1. load object names
    obj_list = read_objlist()
    
    # 2. set tote half size
    tote_length_2 = 0.32 / 2  # x
    tote_width_2  = 0.56 / 2 # y
    tote_height_2 = 0.18 / 2 # z
    
    
    obj_in_bin = []
    obj_ids = []
    tote_center = poseTransform([0,0,0,0,0,0,1], '/tote' , '/map', listener)
    num_obj = 12
    for it_obj in xrange(num_obj):
        obj_id = random.choice(obj_list)
        
        obj_pose = copy.deepcopy(tote_center)
        obj_pose[0] += random.uniform(-tote_length_2, tote_length_2)
        obj_pose[1] += random.uniform(-tote_width_2,  tote_width_2)
        obj_pose[2] += random.uniform(0, tote_height_2 *2 )
        obj_pose[3:7] = tfm.random_quaternion(rand=None).tolist()
        
        obj_dim =  get_obj_dim(obj_id) 
        
        score = random.uniform(0.0 , 1.0)
        obj_ids.append(obj_id)
        obj_in_bin.append({0: obj_id, 1: obj_pose, 'label': obj_id, 'pose': obj_pose, 
                           'pca': obj_pose, 'latent': obj_dim, 'score': score,
                           'mean': obj_pose[0:3], 
                           'rangeX': [obj_pose[0]-0.1, obj_pose[0]+0.1], 
                           'rangeY': [obj_pose[1]-0.1, obj_pose[1]+0.1], 
                           'rangeZ': [obj_pose[2]-0.05, obj_pose[2]+0.05]})
    
    visualizeObjectsDict(obj_in_bin, br, 'map')
    return obj_ids, obj_in_bin

def case_generator(binNum, br):
    # 1. load object names
    obj_list = read_objlist()
    
    # 2. prepare figure
    fig = plt.figure()
    
    ax = fig.add_subplot(111, projection='3d')
    sides = [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
    distFromShelf = 0.05
    (binMouth,  bin_height, bin_width) = getBinMouth(distFromShelf, binNum)
    bin_depth = 0.43
    r = [-0.5,0.5]
    X, Y = np.meshgrid(r, r)
    ax.plot_surface(bin_depth*X+bin_depth/2, bin_width/2+bin_width*Y,bin_height, alpha=0.5, color = 'y')
    ax.plot_surface(bin_depth*X+bin_depth/2, bin_width/2+bin_width*Y,0, alpha=0.5, color = 'y')
    ax.plot_surface(bin_depth*0.5+bin_depth/2, bin_width/2-bin_width*X,bin_height*(0.5+Y), alpha=0.5, color = 'y')
    ax.plot_surface(-bin_depth*0.5+bin_depth/2, bin_width/2-bin_width*X,bin_height*(0.5+Y), alpha=0.5, color = 'y')
    
    # 3. 
    # distribution_obj = [1, 1, 2, 2, 2, 3, 3 ,3, 4, 4, 4, 5, 5, 6, 6, 7] #, 7,8,8,9,10] #distriubtion of shelfs
    distribution_obj = [2, 2, 2, 3, 3 ,3, 3, 4, 4, 6, 6, 8] 
    
    num_obj =  random.choice(distribution_obj)
    print 'Number of objects in the bin: ', num_obj
    configuration_found = False
    while not configuration_found:
        obj_in_bin = []
        #Tria objects randomly
        for it_obj in range(num_obj):
            obj_in_bin.append(random.choice(obj_list))
        
        num_attempts = 10
        for attempt in range(num_attempts):
            is_possible = True
            #Assign pos to the objects
            obj_x = []
            obj_y = []
            obj_ori = []
            obj_vertical_dim = []
            obj_l1 = []
            obj_l2 = []
            obj_l3 = []
            for obj in obj_in_bin:
                obj_dim = get_obj_dim(obj)
                #print obj, ' obj_dim: ', obj_dim
                z_dim = random.randint(0,2)
                while bin_height < obj_dim[z_dim]:
                    z_dim = random.randint(0,2)
                else:
                    obj_vertical_dim.append(z_dim)
                
                obj_l3.append(obj_dim[z_dim])
                
                l1 = obj_dim[(z_dim+1)%3]
                l2 = obj_dim[(z_dim+2)%3]
                #print obj_in_bin, ' dimensions: ', l1, l2, obj_dim[z_dim] 
                obj_l1.append(l1)
                obj_l2.append(l2)
                x = random.uniform(0, bin_depth)
                y = random.uniform(0, bin_width)
                orientation = random.uniform(0, 360)/180*math.pi
                treshold = 0.7
                if random.uniform(0,1) < treshold:
                    orientation = 0
                inside = True
                for side1 in range(0, 4):
                    x1 = rotation_x(orientation,l1*sides[side1][0], l2*sides[side1][1])+x
                    y1 = rotation_y(orientation,l1*sides[side1][0], l2*sides[side1][1])+y
                    inside = inside and (x1 > 0 and x1 < bin_depth) and (y1 > 0 and y1 < bin_width)
                while not inside:                        
                    x = random.uniform(0, bin_depth)
                    y = random.uniform(0, bin_width)
                    orientation = random.uniform(0, 360)/180*math.pi
                    if random.uniform(0,1) < treshold:
                        orientation = 0
                    inside = True
                    for side1 in range(0, 4):
                        x1 = rotation_x(orientation,l1*sides[side1][0], l2*sides[side1][1])+x
                        y1 = rotation_y(orientation,l1*sides[side1][0], l2*sides[side1][1])+y
                        inside = inside and (x1 > 0 and x1 < bin_depth) and (y1 > 0 and y1 < bin_width)
                obj_x.append(x)  #TODO: do rotation functions
                obj_y.append(y)
                obj_ori.append(orientation)
            for it_obj1 in range(0, num_obj):
                if not is_possible:
                    break
                for it_obj2 in range(it_obj1+1, num_obj):
                    if not is_possible:
                        break
                    for side1 in range(0, 4):
                        if not is_possible:
                            break
                        l1 = obj_l1[it_obj1]
                        l2 = obj_l2[it_obj1]
                        orientation = obj_ori[it_obj1]
                        x1 = rotation_x(orientation,l1*sides[side1][0], l2*sides[side1][1])+obj_x[it_obj1]
                        y1 = rotation_y(orientation,l1*sides[side1][0], l2*sides[side1][1])+obj_y[it_obj1]
                        x2 = rotation_x(orientation,l1*sides[(side1+1)%4][0], l2*sides[(side1+1)%4][1])+obj_x[it_obj1]
                        y2 = rotation_y(orientation,l1*sides[(side1+1)%4][0], l2*sides[(side1+1)%4][1])+obj_y[it_obj1]
                        for side2 in range(0, 4):
                            l1 = obj_l1[it_obj2]
                            l2 = obj_l2[it_obj2]
                            orientation = obj_ori[it_obj2]
                            x3 = rotation_x(orientation,l1*sides[side2][0], l2*sides[side2][1])+obj_x[it_obj2]
                            y3 = rotation_y(orientation,l1*sides[side2][0], l2*sides[side2][1])+obj_y[it_obj2]
                            x4 = rotation_x(orientation,l1*sides[(side2+1)%4][0], l2*sides[(side2+1)%4][1])+obj_x[it_obj2]
                            y4 = rotation_y(orientation,l1*sides[(side2+1)%4][0], l2*sides[(side2+1)%4][1])+obj_y[it_obj2]
                            #print 'x1, y1, x2, y2, x3, y3, x4, y4: ', x1, y1, x2, y2, x3, y3, x4, y4
                            #print segment_intersection(x1, y1, x2, y2, x3, y3, x4, y4)
                            
                            #print 'y2', y2
                            #print 'x3', x3
                            if segment_intersection(x1, y1, x2, y2, x3, y3, x4, y4):
                                is_possible = False
                                break
                x1_inf = obj_x[it_obj1]
                y1_inf = obj_y[it_obj1]
                x2_inf = 1000
                y2_inf = 1000
                count_intersection = 0
                for it_obj2 in range(0, num_obj):
                    if not is_possible:
                        break
                    if it_obj1 != it_obj2:
                        for side2 in range(0, 4):
                            l1 = obj_l1[it_obj2]
                            l2 = obj_l2[it_obj2]
                            orientation = obj_ori[it_obj2]
                            x3 = rotation_x(orientation,l1*sides[side2][0], l2*sides[side2][1])+obj_x[it_obj2]
                            y3 = rotation_y(orientation,l1*sides[side2][0], l2*sides[side2][1])+obj_y[it_obj2]
                            x4 = rotation_x(orientation,l1*sides[(side2+1)%4][0], l2*sides[(side2+1)%4][1])+obj_x[it_obj2]
                            y4 = rotation_y(orientation,l1*sides[(side2+1)%4][0], l2*sides[(side2+1)%4][1])+obj_y[it_obj2]
                            if segment_intersection(x1_inf, y1_inf, x2_inf, y2_inf, x3, y3, x4, y4):
                                count_intersection += 1
                        if count_intersection%2 == 1:
                            is_possible = False
                            break
            if is_possible == True:
                configuration_found = True
                object_poses = []
                obj_ids = []
                for it_obj in range(num_obj):
                    # if it_obj == 0:
                        # color = 'r'
                        # rgba = [0.1,0.8,0.1,1]
                    # else:
                        # color = 'b'
                        # rgba = [0.7,0.1,0.2,1]
                    rgba = None
                    color = 'r'
                    plot_shape(obj_l1[it_obj], obj_l2[it_obj],obj_l3[it_obj],obj_x[it_obj],obj_y[it_obj],obj_ori[it_obj], ax, color)
                    if obj_vertical_dim[it_obj] == 2: #TODO add -1 sign to flip +180 degrees
                        ori = tfm.quaternion_from_euler(*[0,0,obj_ori[it_obj]]) #obj_ori[it_obj]
                    elif obj_vertical_dim[it_obj] == 1:
                        ori = tfm.quaternion_from_euler(*[math.pi/2,0,math.pi/2+obj_ori[it_obj]])
                    else:
                        ori = tfm.quaternion_from_euler(*[0,math.pi/2,math.pi/2+obj_ori[it_obj]])
                        #print ori
                    #print obj_in_bin[it_obj], ori
                    if  obj_x[it_obj] > 0.2:
                        obj_l3[it_obj] -= 2*0.02
                    else:
                        obj_l3[it_obj] -= 2*obj_x[it_obj]*0.02/0.2
                    object_pose = {0: obj_in_bin[it_obj], 1: [ obj_x[it_obj], obj_y[it_obj]-bin_width/2, obj_l3[it_obj]/2,ori[0],ori[1],ori[2],ori[3]] } 
                    obj_pose = object_pose[1]
                    object_pose['label'] = object_pose[0]
                    object_pose['pose'] = obj_pose
                    object_pose['pca'] = obj_pose
                    object_pose['latent'] = get_obj_dim(obj)
                    object_pose['mean'] = obj_pose[0:3]
                    object_pose['rangeX'] = [obj_pose[0]-0.1, obj_pose[0]+0.1]
                    object_pose['rangeY'] = [obj_pose[1]-0.1, obj_pose[1]+0.1]
                    object_pose['rangeZ'] = [obj_pose[2]-0.05, obj_pose[2]+0.05]
                    obj_ids.append(obj_in_bin[it_obj])
                    object_poses.append(object_pose)
                    visualizeObjectsDict(object_poses , br, 'bin'+str(binNum), rgba)
                    #visualizeObjectsCloud([[ obj_in_bin[it_obj], [ obj_x[it_obj], obj_y[it_obj]-bin_width/2, obj_l3[it_obj]/2,ori[0],ori[1],ori[2],ori[3]] ]] , br, 'bin'+str(binNum))
                    rospy.sleep(0.1)
                #plt.show()
                break
    result = {}
    result["poses"] = object_poses
    result["bin_id"] = binNum
    result["goal"] = 0
    result["objects_id"] = obj_ids
    return result

def cleanup_rviz():
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.sleep(1)
    marker = Marker()
    marker.action = 3
    pub.publish(marker)
    rospy.sleep(1)

if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    cleanup_rviz()


    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    objects_in_shelf = {}
    pose_in_shelf = {}
    
    objects_in_tote = {}
    pose_in_tote = {}
    
    for binNum in xrange(0, 12):
        result = case_generator(binNum=binNum, br=br)
        
        letter = chr(binNum + ord('A'))
        objects_in_shelf["bin_%s" % (letter)] = result["objects_id"]
        pose_in_shelf["bin_%s" % (letter)] = result["poses"]
    
    objects_in_tote, pose_in_tote = case_generator_tote(listener=listener, br=br)
    
    outfilename = 'apc_stow_task.json' 
    with open(outfilename, 'w') as outfile:
        json.dump({'bin_contents': objects_in_shelf, 'tote_contents': objects_in_tote} , outfile, sort_keys = True, indent = 4, ensure_ascii=False)
    
    outfilename = 'apc_stow_task.pose.json'
    with open(outfilename, 'w') as outfile:
        json.dump({'bin_contents': pose_in_shelf, 'tote_contents': pose_in_tote} , outfile, sort_keys = True, indent = 4, ensure_ascii=False)
    
