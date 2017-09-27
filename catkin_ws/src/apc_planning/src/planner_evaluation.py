#!/usr/bin/env python

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
import grasp16
import suction_down16
import scoop
import suction_side16
import suction_forward16
from fake_inputbag import Featurize
from learned_heuristic import ComputeProbPrimitive, APCScenario_from_poses_list
def plot_shape(l1, l2,l3, x, y, theta, ax, color):
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

def ccw(Ax, Ay,Bx, By,Cx, Cy):
    return (Cy-Ay)*(Bx-Ax) > (By-Ay)*(Cx-Ax)

def segment_intersection(Ax, Ay,Bx, By,Cx, Cy,Dx, Dy):
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

def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False

def ReadParameters(filename):
    global FeatCoef
    param_file_path = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/alpha_cube/'+filename

    try:
        with open(param_file_path) as data_file:
            FeatCoef = json.load(data_file)
    except:
        print '[A-C] loading parameter file error:', filename
        print '[A-C] you can copy the file from mcube-002'
        exit(0)

    for key in FeatCoef: #Is this doing anything??
        print key
        key = str(key)
        print key
        for subkey in FeatCoef[key]:
            subkey = str(subkey)

def ReadParameters2(filename):
    global FeatCoef2
    param_file_path = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/alpha_cube/'+filename

    try:
        with open(param_file_path) as data_file:
            FeatCoef2 = json.load(data_file)
    except:
        print '[A-C] loading parameter file error:', filename
        print '[A-C] you can copy the file from mcube-002'
        exit(0)

    for key in FeatCoef2: #Is this doing anything??
        print key
        key = str(key)
        print key
        for subkey in FeatCoef2[key]:
            subkey = str(subkey)

def ReadParameters3(filename):
    global FeatCoef3
    param_file_path = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/alpha_cube/'+filename

    try:
        with open(param_file_path) as data_file:
            FeatCoef3 = json.load(data_file)
    except:
        print '[A-C] loading parameter file error:', filename
        print '[A-C] you can copy the file from mcube-002'
        exit(0)

    for key in FeatCoef3: #Is this doing anything??
        print key
        key = str(key)
        print key
        for subkey in FeatCoef3[key]:
            subkey = str(subkey)

def case_generator(binNum, listener):
    global FeatCoef
    global FeatCoef2
    global FeatCoef3
    lFilename = 'june24.json'
    ReadParameters(lFilename)
    ReadParameters2('A.json')
    ReadParameters3('B.json')

    jsonFilename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/object_data/allnewobjects_without_rolodex.json'
    with open(jsonFilename) as data_file:
        obj_list = json.load(data_file) #dimension in mm


    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')
    sides = [[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]
    distFromShelf = 0.05
    (binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    bin_depth = 0.43
    r = [-0.5,0.5]
    X, Y = np.meshgrid(r, r)
    ax.plot_surface(bin_depth*X+bin_depth/2, bin_width/2+bin_width*Y,bin_height, alpha=0.5, color = 'y')
    ax.plot_surface(bin_depth*X+bin_depth/2, bin_width/2+bin_width*Y,0, alpha=0.5, color = 'y')
    #ax.plot_surface(bin_depth*X+bin_depth/2, bin_width/2-bin_width*0.5,bin_height*(0.5+Y), alpha=0.5, color = 'y')
    ax.plot_surface(bin_depth*0.5+bin_depth/2, bin_width/2-bin_width*X,bin_height*(0.5+Y), alpha=0.5, color = 'y')
    ax.plot_surface(-bin_depth*0.5+bin_depth/2, bin_width/2-bin_width*X,bin_height*(0.5+Y), alpha=0.5, color = 'y')
    '''

    ax.plot_surface(rotate_x(0.5,X)+x, rotate_y(0.5,X)+y,l3*(0.5+Y), alpha=0.5, color = color)
    ax.plot_surface(rotate_x(-0.5,X)+x, rotate_y(-0.5,X)+y,l3*(0.5+Y), alpha=0.5, color = color
    '''
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    distribution_obj = [1,1, 2, 2, 2, 3, 3 ,3, 4, 4,4, 5,5,6,6, 7] #, 7,8,8,9,10] #distriubtion of shelfs
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
                for it_obj in range(num_obj):
                    if it_obj == 0:
                        color = 'r'
                        rgba = [0.1,0.8,0.1,1]
                    else:
                        color = 'b'
                        rgba = [0.7,0.1,0.2,1]
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
                    object_pose = {}
                    object_pose[0] = object_pose['label'] = obj_in_bin[it_obj]
                    object_pose[1] = object_pose['pose'] = poseTransform([ obj_x[it_obj], obj_y[it_obj]-bin_width/2, obj_l3[it_obj]/2,ori[0],ori[1],ori[2],ori[3]], '/bin'+str(binNum), '/map', listener)
                    object_pose['score'] = 1.
                    object_pose['pca'] = object_pose['mean'] = object_pose['pose'][0:3]
                    object_pose['latent'] = [obj_l1[it_obj]/4, obj_l2[it_obj]/4, obj_l3[it_obj]/4]
                    object_pose['rangeX'] = [object_pose['mean'][0] - obj_l1[it_obj]/4, object_pose['mean'][0] + obj_l1[it_obj]/4]
                    object_pose['rangeY'] = [object_pose['mean'][1] - obj_l2[it_obj]/4, object_pose['mean'][0] + obj_l2[it_obj]/4]
                    object_pose['rangeZ'] = [object_pose['mean'][1] - obj_l3[it_obj]/4, object_pose['mean'][0] + obj_l3[it_obj]/4]


                    object_poses.append(object_pose)
                    visualizeObjects([[ obj_in_bin[it_obj], [ obj_x[it_obj], obj_y[it_obj]-bin_width/2, obj_l3[it_obj]/2,ori[0],ori[1],ori[2],ori[3]] ]] , br, '/bin'+str(binNum), rgba)
                    #visualizeObjectsCloud([[ obj_in_bin[it_obj], [ obj_x[it_obj], obj_y[it_obj]-bin_width/2, obj_l3[it_obj]/2,ori[0],ori[1],ori[2],ori[3]] ]] , br, 'bin'+str(binNum))
                    rospy.sleep(0.1)
                #plt.show()
                break


    result = {}
    result["poses"] = object_poses
    result["bin_id"] = binNum
    result["goal"] = object_poses[0][0]
    primitives = ["suction-down", "suction-front", "suction-side", "grasping", "scooping"] #"push_front", "push_side"
    common_answer = False
    store = True

    strategies = {'grasping' : grasp16.grasp,
                  'suction-down' : suction_down16.suction_down,
                  'scooping' : scoop.scoop,
                  'suction-side' : suction_side16.suction_side,
                  'suction-front' : suction_forward16.suction_forward}


    ############################
    ### Planner expectations ###
    ############################
    bin_contents = [pose['label'] for pose in object_poses]
    bin_contents.sort()
    disc_factor = 2
    range_x = int(disc_factor*47)#47
    range_y = int(disc_factor * 100 * bin_width)#int(100 * bin_width)
    scene = APCScenario_from_poses_list(result["goal"], bin_contents, object_poses, binNum, listener)
    grids = np.zeros((scene.num_objects, range_x, range_y))
    features = Featurize(scene, grids, False, silent = False)
    for feature in sorted(features):
        print feature, ' '*(30-len(feature)), '%.2f' % features[feature]
    for primitive in strategies:
        expectation = (ComputeProbPrimitive(primitive, features, def_FeatCoef=FeatCoef))*4
        expectation2 = (ComputeProbPrimitive(primitive, features, def_FeatCoef=FeatCoef2, num_explain = 5))*4
        expectation3 = (ComputeProbPrimitive(primitive, features, def_FeatCoef=FeatCoef3, num_explain = 5))*4
        print primitive,': ', '%.1f' % expectation2, ' --> ', '%.1f' % expectation, ' --> ', '%.1f' % expectation3
    pauseFunc(True)

    return store, result




if __name__=='__main__':
    rospy.init_node('case_generator', anonymous=True)
    pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    rospy.sleep(1)
    marker = Marker()
    marker.action = 3
    pub.publish(marker)
    rospy.sleep(1)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    print 'Recall to have an empty RViz (wihtout markers) before starting the experiment.'
    lab_members = ['peter', 'maria', 'ferran', 'nikhil', 'frank', 'orion', 'nima', 'roman', 'elliott', 'alberto', 'random_guy', 'debug']
    print 'Available users: ', lab_members
    user = raw_input('Username: ').lower()
    while not user in lab_members:
        user = raw_input('Incorrect username. Try again please: ').lower()

    experiments_recorded = []

    for binNum in range(0,12):
        store, result = case_generator(binNum=binNum, listener=listener)
        if store:
            result["user"] = user
            experiments_recorded.append(result)

    '''#Time to print it to a json file!
    now = datetime.now()

    outfilename = os.environ['APC_BASE']+'/doc/human_labeling/%s_%s-%s-%s-%s-%s.json' % (user, now.month, now.day, now.hour, now.minute, now.second)
    with open(outfilename, 'w') as outfile:
        json.dump(experiments_recorded, outfile, sort_keys = True, indent = 4, ensure_ascii=False)'''
    answer = raw_input('Do you want to continue? (y/n)')
    while answer != 'y' and answer != 'n':
        answer = raw_input('Answer only \'y\' or \'n\' : ').lower()
    if answer == 'y':
        continue_experiment = True
    else:
        continue_experiment = False
        print 'Thank you! Please commit your experiments :-)'
    while continue_experiment:
        rospy.sleep(1)
        marker = Marker()
        marker.action = 3
        pub.publish(marker)
        rospy.sleep(1)
        experiments_recorded = []
        #rospy.init_node('case_generator_continued', anonymous=True)
        for binNum in range(0,12):
            store, result = case_generator(binNum=binNum, listener=listener)
            if store:
                result["user"] = user
                experiments_recorded.append(result)

        '''#Time to print it to a json file!
        now = datetime.now()

        outfilename = os.environ['APC_BASE']+'/doc/human_labeling/%s_%s-%s-%s-%s-%s.json' % (user, now.month, now.day, now.hour, now.minute, now.second)
        with open(outfilename, 'w') as outfile:
            json.dump(experiments_recorded, outfile, sort_keys = True, indent = 4, ensure_ascii=False)'''
        answer = raw_input('Do you want to continue? (y/n)')
        while answer != 'y' and answer != 'n':
            answer = raw_input('Answer only \'y\' or \'n\' : ').lower()
        if answer == 'y':
            continue_experiment = True
        else:
            continue_experiment = False
            print 'Thank you! Please commit your experiments :-)'
