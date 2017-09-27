#!/usr/bin/python

import copy
import tf
from ik.roshelper import *
from ik.helper import *
import rospy
import numpy as np
import math
import rosbag
import subprocess, os, signal
import errno
#Others needed
from ik.helper import get_obj_dim
#msg's
from apc_planning.msg import APC_Scenario
#Gaussian Process
from scipy import stats
from sklearn.gaussian_process import GaussianProcess
from matplotlib import pyplot as pl
from matplotlib import cm

global BinH

#General parameters
num_objects = 5

#Broadcaster for scene generation
#br = tf.TransformBroadcaster()

#Copied from https://github.mit.edu/mCube/pnpush/blob/master/catkin_ws/src/pnpush_config/src/config/helper.py
def start_ros_bag(bagfilename, topics, dir_save_bagfile):
    subprocess.Popen('rosbag record -q -O %s %s' % (bagfilename, " ".join(topics)) , shell=True, cwd=dir_save_bagfile)

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)

def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

def PlotGP(gp,X,y):
    # Standard normal distribution functions
    phi = stats.distributions.norm().pdf
    PHI = stats.distributions.norm().cdf
    PHIinv = stats.distributions.norm().ppf

    # A few constants
    lim = 8

    # Evaluate real function, the prediction and its MSE on a grid
    res = 50
    x1, x2 = np.meshgrid(np.linspace(- lim, lim, res),
                         np.linspace(- lim, lim, res))
    xx = np.vstack([x1.reshape(x1.size), x2.reshape(x2.size)]).T

#    y_true = g(xx)
    y_pred, MSE = gp.predict(xx, eval_MSE=True)
    sigma = np.sqrt(MSE)
#    y_true = y_true.reshape((res, res))
    y_pred = y_pred.reshape((res, res))
    sigma = sigma.reshape((res, res))
    k = PHIinv(.975)

    # Plot the probabilistic classification iso-values using the Gaussian property
    # of the prediction
    fig = pl.figure(1)
    ax = fig.add_subplot(111)
    ax.axes.set_aspect('equal')
    pl.xticks([])
    pl.yticks([])
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    pl.xlabel('$x_1$')
    pl.ylabel('$x_2$')

    cax = pl.imshow(np.flipud(PHI(- y_pred / sigma)), cmap=cm.gray_r, alpha=0.8,
                    extent=(- lim, lim, - lim, lim))
    norm = pl.matplotlib.colors.Normalize(vmin=0., vmax=0.9)
    cb = pl.colorbar(cax, ticks=[0., 0.2, 0.4, 0.6, 0.8, 1.], norm=norm)
    cb.set_label('${\\rm \mathbb{P}}\left[\widehat{G}(\mathbf{x}) \leq 0\\right]$')
    print 'y', y
    pl.plot(X[y <= 0, 0], X[y <= 0, 1], 'r.', markersize=12)

    pl.plot(X[y > 0, 0], X[y > 0, 1], 'b.', markersize=12)

#    cs = pl.contour(x1, x2, y_true, [0.], colors='k', linestyles='dashdot')

    cs = pl.contour(x1, x2, PHI(- y_pred / sigma), [0.025], colors='b',
                    linestyles='solid')
    pl.clabel(cs, fontsize=11)

    cs = pl.contour(x1, x2, PHI(- y_pred / sigma), [0.5], colors='k',
                    linestyles='dashed')
    pl.clabel(cs, fontsize=11)

    cs = pl.contour(x1, x2, PHI(- y_pred / sigma), [0.975], colors='r',
                    linestyles='solid')
    pl.clabel(cs, fontsize=11)

    pl.show()

def BagToArrays(bag,a,b):
    X = [[] for i in range(num_objects)]
    y = [[] for i in range(num_objects)]
    NB = [[] for i in range(num_objects)]
    for topic, msg, t in bag.read_messages(topics=['/Ferran/scenes']):
        if msg.num_objects==1:
            #X.append([[msg.objects[0].ObjId, msg.objects[0].position[3:7]]])
            X[msg.objects[0].ObjId].append([msg.objects[0].position[i] for i in range(a,b)])
            y[msg.objects[0].ObjId].append(msg.result)
            NB[msg.objects[0].ObjId].append(msg.objects[0].binId)
    print X
    X = np.array(X)
    for i in range(num_objects):
        X[i] = np.array(X[i])
        y[i] = np.array(y[i])
    y = np.array(y)
    NB = np.array(NB)
    return [X,y,NB]

def BasicOneObjectSceneGeneration(bag):
    #Given a bag with the experiments done, suggests objects and
    #quaternions that have to be tried.
    threshold_per_object = 4
    num_goal_quaternions = 3
    GoalQuaternions = np.array([[1,0,0,0],
                                [0.7071,0.,0.7071,0.],
                                [0,0,1,0],
                                [0.7071,0,-0.7071,0.],
                                [0.7071,0,0,0.7071],
                                [0.,0.7071,0.7071,0.],
                                [0.7071,0.,0.,-0.7071]])
    GoalMeasures = np.ones(num_goal_quaternions)
    [X,y,NB] = BagToArrays(bag,3,7)
    print 'X',X
    print 'y',y

    for i in range(num_objects):
        if len(X[i].shape)==0 or X[i].shape[0]<threshold_per_object:
            print 'Object %s should be tried %s more times' % (i,threshold_per_object-X[i].shape[0])
        if len(X[i].shape)==0:
            continue
        for j in range(num_goal_quaternions):
            count = 0.
            for k in range(X[i].shape[0]):
                count = count+1
                for l in range(X[i].shape[1]):
                    count = count - GoalQuaternions[j][l]*X[i][k][l]
            if count<GoalMeasures[j]:
                print '.....lacking %s measures of quat %s' % (GoalMeasures[j]-count,GoalQuaternions[j])

def VisualizeHeightMap(H, G,m_per_x, m_per_y):
    [range_x,range_y] = H.shape
    sth_interesting = False
    eps = 0.001
    #np.set_printoptions(threshold='nan')
    #print 'H: ', H
    #print 'G: ', G
    #D = copy.deepcopy(G)
    #for x in range(np.shape(D)[0]):
    #    for y in range(np.shape(D)[1]):
    #        D[x,y]-=H[x,y]
    #print 'D: ', D
    global empty_bin_H
    if empty_bin_H==None:
        print 'Cry a little  because theres not empty_bin_H'
    for x in xrange(range_x-1,-1,-1):
        if empty_bin_H == None:
            minimum_height = max(-0.02,-x*m_per_x*0.02/0.2)+eps
        else:
            minimum_height = empty_bin_H[x,0]
        if x < 0.5*range_x:
            sth_interesting = True
        st = str(x%10)
        for y in range(int(range_y)-1,-1,-1):
            if H[x,y]>minimum_height or G[x,y]>minimum_height:
#                print H[x,y]-minimum_height, G[x,y]-minimum_height
                if abs(G[x,y]-H[x,y])>eps:
                    if H[x,y]<minimum_height+eps:
                        h = int(100*G[x,y]+0.5)
                        if h>9:
                            st = st + '\033[92m' + '#' + '\033[0m'
                        elif h<0:
                            st = st + '\033[92m' + '=' + '\033[0m'
                        else:
                            st = st + '\033[92m' + chr(ord('0')+h) + '\033[0m'
                    elif H[x,y]>G[x,y]+eps:
                        st = st +  '\033[92m' + 'L' + '\033[0m'
                    else:
                        st = st +  '\033[92m' + 'T' + '\033[0m'
                else:
                    h = int(100*H[x,y]+0.5)
                    if h>9:
                        st = st + '\033[94m' + '#' + '\033[0m'
                    elif h<0:
                        st = st + '\033[94m' + '=' + '\033[0m'
                    else:
                        st = st + '\033[94m' + chr(ord('0')+h) + '\033[0m'
                sth_interesting = True
            else:
                st = st +  '.'
        if sth_interesting:
            print st
    st = ' '
    for y in range(range_y-1,-1,-1):
        st = st + str(y%10)
    print st

def Featurize(scene, grids, computed_grids, silent = True, disc_factor = 2):
    vision_score_threshold = 0.15
    if not silent:
        print 'Goal Object: ', scene.goal_object
    if not silent:
        print scene
    #TODO: shift and rotate everything to bin coordinates
    [right, left, back,front,bottom,top] = get_bin_cnstr()[scene.objects[0].binId]
    bin_width = left-right
    bin_depth = front-back
    bin_height = top-bottom
    #I think right now axis are parallel to bin, not map
    range_x = int(disc_factor*47)#47
    range_y = int(disc_factor*100*bin_width)#int(100*bin_width)
    ratio_x = bin_depth/range_x
    ratio_y = bin_width/range_y
    H = np.zeros((range_x, range_y))
    for x in range(range_x):
        for y in range(range_y):
            H[x,y] = max(-0.02,-x*ratio_x*0.02/0.2)
    global empty_bin_H
    empty_bin_H = copy.deepcopy(H)
    print grids.shape, ' vs ', scene.num_objects, range_x, range_y
    assert (grids.shape == (scene.num_objects,range_x,range_y))
    #fill H
    for i in range(scene.num_objects):
        if i==scene.goal_object:
            #TODO: maybe do something
            continue

        object_size = get_obj_dim(scene.objects[i].ObjId)
        # We don't do anything about id
        for x in range(range_x):      #we should use a corner, as we did with the object, aixo nomes n'agafa 4, no 5
            for y in range(range_y):
                if not computed_grids:
                    h_obst = in_or_out(grid_x = x,grid_y = y,
                            m_per_x = ratio_x, m_per_y = ratio_y,
                            position = scene.objects[i].position[0:3],
                            orientation = scene.objects[i].position[3:7],
                            dim = object_size)
                    grids[i][x][y] = h_obst
                else:
                    h_obst = grids[i][x][y]
                if scene.objects[i].position[0] < -1: #Object not in scene
                    h_obst = -10
                if scene.objects[i].score < vision_score_threshold: #Too low vision score
                    h_obst = -10
                #if h_obst>0:
                #    print '(', x, ',' , y, ')'
                H[x,y] = max(H[x,y], h_obst)
    INF = 1000000
    min_goal_col = INF*np.ones(range_y)
    max_goal_col = -INF*np.ones(range_y)
    min_goal_row = INF*np.ones(range_x)
    max_goal_row = -INF*np.ones(range_x)
    G = copy.deepcopy(H) #height-map including the goal_object.

    object_size = get_obj_dim(scene.objects[scene.goal_object].ObjId)

    top_height = -10

    for x in range(range_x):
        for y in range(range_y):
            if not computed_grids:
                h_obst = in_or_out(grid_x = x,grid_y = y,
                        m_per_x = ratio_x, m_per_y = ratio_y,
                        position = scene.objects[scene.goal_object].position[0:3],
                        orientation = scene.objects[scene.goal_object].position[3:7],
                        dim=object_size)

                grids[scene.goal_object][x][y] = h_obst
            else:
                h_obst = grids[scene.goal_object][x][y]
            if scene.objects[scene.goal_object].position[0] < -1:
                h_obst = -10
            if h_obst<= empty_bin_H[x,y]+0.001:
                continue
            #print x, y, h_obst
            G[x,y] = max(G[x,y], h_obst)
            top_height = max(top_height, G[x,y])
            #Now we know we are inside the goal_object.
            min_goal_col[y] = min(min_goal_col[y], x)
            max_goal_col[y] = max(max_goal_col[y], x)
            min_goal_row[x] = min(min_goal_row[x], y)
            max_goal_row[x] = max(max_goal_row[x], y)
    top_height = abs(top_height)
    grasp_height = top_height/2
    height_goal = scene.objects[scene.goal_object].position[2]

    if not silent:
        VisualizeHeightMap(H,G,m_per_x = ratio_x, m_per_y = ratio_y)
    #np.set_printoptions(threshold='nan')
    #print 'H: ', H
    #print 'G: ', G
    def compute_some_features(prefix):
        #Calculating things with respect to the goal:
        #TODO: update dist-x with walls
        dist_left = 0.08/ratio_y
        dist_right = 0.08/ratio_y
        dist_behind = 0.08/ratio_x
        dist_front = 0.08/ratio_x
        dist_wall = 0.08/ratio_y
        clutter_front = 0
        clutter_front_grasp = 0
        clutter_front_suction_down = 0
        clutter_behind = 0
        clutter_close_behind = 0
        clutter_top = 0
        clutter_below = 0
        number_front = 0
        number_behind = 0
        top = 0
        below = 0
        area = 0 #Area of the goal object.
        max_max_col = -INF
        min_min_col = INF
        max_max_row = -INF
        min_min_row = INF
        global empty_bin_H
        #TODO: watch out for ratio factors.
        #TODO: maybe good to make the object bigger to include the gripper?
        # 'min_goal_col: ', min_goal_col
        #print 'max_goal_col: ', max_goal_col
        for y in range(range_y):
            if min_goal_col[y] < INF-1:
                number_front = number_front+min_goal_col[y]
                area = area + max_goal_col[y]-min_goal_col[y]+1.0
                number_behind += min( 0.05/ratio_x,range_x-max_goal_col[y]-1)
                #dist_front = min(dist_front, min_goal_col[y])  #Do we want that dist_front measures distance to the begining?
                #dist_behind = min(dist_behind, range_x - 1 - max_goal_col[y])
            min_min_col = min(min_min_col, min_goal_col[y])
            max_max_col = max(max_max_col, max_goal_col[y])
        for x in range(range_x):
            if min_goal_row[x] < INF:
                dist_wall = min(dist_wall, min(range_y-1-max_goal_row[x], min_goal_row[x]))
                #dist_left = min(dist_left, min_goal_row[x])
                #dist_right = min(dist_right, range_y - 1 - max_goal_row[x])
            min_min_row = min(min_min_row, min_goal_row[x])
            max_max_row = max(max_max_row, max_goal_row[x])

        depth = max_max_col-min_min_col
        width = max_max_row-min_min_row

        #No gripper features:
        for x in range(range_x):
            for y in range(range_y):
                if H[x,y] < empty_bin_H[x,y] + 0.001:
                    continue #Ensuring there's an object there.
                if min_goal_col[y] < INF and x < min_goal_col[y]: #Front
                    if H[x,y] > 0.5*grasp_height:
                        dist_front = min(dist_front, min_goal_col[y] - x -1)
                    if H[x,y] > 0:
                        clutter_front += 1.0
                    if H[x,y] > grasp_height:
                        clutter_front_grasp += 1.0
                    if H[x,y] > top_height-0.01:
                        clutter_front_suction_down += 1.0
                if min_goal_col[y] < INF and x > max_goal_col[y]:
                    dist_behind = min(dist_behind, x - max_goal_col[y]-1)
                    clutter_behind = clutter_behind + 1.0
                    if x < max_goal_col[y] + 0.05/ratio_x: #5 cm Behind
                        clutter_close_behind += 1.0
                if min_goal_row[x] < INF and y < min_goal_row[x]: #Left
                    dist_left = min(dist_left, min_goal_row[x]-y-1)
                if min_goal_row[x] < INF and y > max_goal_row[x]: #Right
                    dist_right = min(dist_right, y - max_goal_row[x]-1)
                if min_goal_col[y] < INF and min_goal_row[x] < INF and min_goal_col[y]<=x and x<=max_goal_col[y] \
                    and min_goal_row[x] <=y and y<=max_goal_row[x]: #Top/Below
                    if H[x,y]<=top_height: #Something below
                        below = 1
                        clutter_below = clutter_below + 1.0
                    else:
                        clutter_top = clutter_top + 1.0
                        top = 1
        number_front = min(0.12*0.12/(ratio_x*ratio_y),max(5, max(0.03*0.03/(ratio_x*ratio_y),number_front)))
        clutter_front_suction_down = max(0.,min(1.,clutter_front_suction_down/number_front))
        clutter_front_grasp = max(0.,min(1.,(clutter_front_grasp-clutter_front_suction_down)/number_front))
        clutter_front = max(0.,min(1.,(clutter_front-clutter_front_grasp-clutter_front_suction_down)/number_front))
        number_behind = min(0.12*0.05/(ratio_x*ratio_y),max(5, max(0.03*0.03/(ratio_x*ratio_y),number_behind)))
        clutter_behind = min(1.,float(clutter_behind)/number_behind)
        clutter_close_behind = min(1.,clutter_close_behind/number_behind)
        if area==0:
            print '[Featurize]: area is 0!!!! Maybe discretization is too big?'
            area = 1.0
        diagonal = abs(width) + abs(depth)#max(math.sqrt(width*width+depth*depth),1)
        F_aux = {
            prefix+'dist_min_left_right' : max(0.,min(1.,16*min(dist_left,dist_right)*ratio_y)),
            prefix+'dist_max_left_right' : max(0.,min(1.,16*max(dist_left,dist_right)*ratio_y)),
            prefix+'015dist_wall' : 1.*(dist_wall*ratio_y<0.015),
            prefix+'025dist_wall' : 1.*(dist_wall*ratio_y<0.025),
            prefix+'035dist_wall' : 1.*(dist_wall*ratio_y<0.035),
            prefix+'dist_front' : max(0.,min(1.,16*(dist_front*ratio_x))), #piecewise linear: 0.0,0.0,0.25,0.5,0.75,1.
            prefix+'dist_behind' : max(0.,min(1.,16*(dist_behind*ratio_x))),
            prefix+'clutter_front' : clutter_front,
            prefix+'clutter_front_grasp' : clutter_front_grasp,
            prefix+'clutter_front_suction_down' : clutter_front_suction_down,
            prefix+'clutter_behind' : clutter_behind,
            prefix+'sth_behind': 1.*(clutter_behind>0.15),
            prefix+'clutter_close_behind' : clutter_close_behind,
            prefix+'clutter_top' : min(1.,float(clutter_top)/float(area)),
            prefix+'clutter_inside' : min(1.,float(clutter_top+clutter_below)/area), #TODO: redefine
            prefix+'05_front' : 1.*((clutter_front_grasp + clutter_front_suction_down) > 0.5),
            prefix+'02_front' : 1.*((clutter_front_grasp + clutter_front_suction_down) > 0.2),
            prefix+'00_front' : 1.*((clutter_front_grasp + clutter_front_suction_down) > 0),
            #### ONLY NOT-EMPTY #### TODO
            #### ONLY EMPTY ####
            prefix+'x' : (scene.objects[scene.goal_object].position[0]/bin_depth)*(scene.objects[scene.goal_object].position[0]/bin_depth),
            prefix+'width_depth_ratio' : abs(width)/diagonal,
            #### ERASE ####
            prefix+'clutter_below' : min(1.,clutter_below/area), #erase
            prefix+'dist_left' : max(0.,min(1.,16*dist_left*ratio_y)), #erase
            prefix+'dist_right' : max(0.,min(1.,16*dist_right*ratio_y)), #erase
            prefix+'depth' : abs(depth)/diagonal #erase
        }
        return F_aux
    ######### END OF compute_some_features() ############
    F = {'bias' : 1}
    F.update(compute_some_features(''))



    #HACK: gripper is goes around
    min_goal_col = INF*np.ones(range_y)
    max_goal_col = -INF*np.ones(range_y)
    min_goal_row = INF*np.ones(range_x)
    max_goal_row = -INF*np.ones(range_x)
    conv_y = int(0.02/ratio_y+1.99) #(2+eps) cm
    conv_x = int(0.015/ratio_y+1.99) #(2+eps) cm
    A = np.zeros((range_x, range_y)) #Is there the goal there?
    #print A
    SA = np.zeros((range_x, range_y)) #Cum sum of A
    for x in range(range_x):
        for y in range(range_y):
            if not computed_grids:
                h_obst = in_or_out(grid_x = x,grid_y = y,
                        m_per_x = ratio_x, m_per_y = ratio_y,
                        position = scene.objects[scene.goal_object].position[0:3],
                        orientation = scene.objects[scene.goal_object].position[3:7],
                        dim=object_size)

                grids[scene.goal_object][x][y] = h_obst
            else:
                h_obst = grids[scene.goal_object][x][y]
            if scene.objects[scene.goal_object].position[0] < -1:
                h_obst = -10
            if h_obst<= empty_bin_H[x,y]+0.001:
                continue
            else:
                A[x][y] = 1
    for i in range(len(SA)):
        for j in range(len(SA[i])):
            SA[i][j] = A[i][j]
            if i>0:
                SA[i][j] += SA[i-1][j]
            if j>0:
                SA[i][j] += SA[i][j-1]
            if i>0 and j>0:
                SA[i][j] -= SA[i-1][j-1]
    #print '#'*50
    #print SA
    #pauseFunc(True)
    for i in range(len(SA)):
        print_string = ''
        for j in range(len(SA[i])):
            big_i = min(len(SA)-1,i + conv_x-1)
            small_i = max(0,i - conv_x+1)
            big_j = min(len(SA[0])-1,j + conv_y-1)
            small_j = max(0,j - conv_y+1)
            aux = SA[big_i][big_j]-SA[small_i][big_j]-SA[big_i][small_j]+SA[small_i][small_j]
            if aux>0:
                min_goal_col[j] = min(min_goal_col[j], i)
                max_goal_col[j] = max(max_goal_col[j], i)
                min_goal_row[i] = min(min_goal_row[i], j)
                max_goal_row[i] = max(max_goal_row[i], j)
                #print_string +=  '#' #('%d' % B[i][j])
            #else:
                #print_string += '.'
            #print_string += ('%d' % B[i][j])
        #print print_string
    '''for i in range(len(SA)):
        print_string = ''
        for j in range(len(SA[i])):
            if A[i][j]>0:
                print_string+='#'
            else:
                print_string+='.'
        print print_string'''

    '''#print 'conv_y: ' , conv_y
    copy_min_goal_col = copy.deepcopy(min_goal_col)
    copy_max_goal_col = copy.deepcopy(max_goal_col)
    for y in range(range_y):
        for dy in range(max(0,y-conv_y),min(range_y,y+conv_y+1),1):
            min_goal_col[y] = min(copy_min_goal_col[y],copy_min_goal_col[dy])
            max_goal_col[y] = max(copy_max_goal_col[y],copy_max_goal_col[dy])'''
    F.update(compute_some_features('gripper_'))
    return F
def OneObjectDistance(pos_x, pos_y, X, num_bin, goal_quaternion, reference = '/shelf'):
    #This function is thought in shelf coordinates
    [right, left, back,front,bottom,top] = get_bin_cnstr()[num_bin]
    binWidth = left-right
    binCenter = (left+right)/2.
    binBack = back-front
    distance = 0
    if pos_x==0:
        distance = distance + max(0.,min(left-X[0],X[0]-right)-0.05)/binWidth #Wall
    else:
        distance = distance + abs(binCenter-X[0])/binWidth #Center
    if pos_y == 0:
        distance = distance + max(0., (X[1]-front-0.05)/binBack) #Front
    else:
        distance = distance + max(0., (back-0.05-X[1])/binBack) #Back
    distance = distance/1.4142 #Because yes.
    #distance in quaternion space
    distance = distance + 1
    for l in range(4):
        distance = distance - goal_quaternion[l]*X[l+3]
    return distance

def OneObjectSceneGeneration(bag):
    #Produces a rviz tf's and text files explaining the situation.
    num_bins = 12
    threshold_per_object = 1
    num_goal_quaternions = 3
    GoalQuaternions = np.array([[1,0,0,0],
                                [0.7071,0.,0.7071,0.],
                                [0,0,1,0],
                                [0.7071,0,-0.7071,0.],
                                [0.7071,0,0,0.7071],
                                [0.,0.7071,0.7071,0.],
                                [0.7071,0.,0.,-0.7071]])
    GoalMeasures = np.ones((2,2,num_goal_quaternions))
    [X,y,NB] = BagToArrays(bag,0,7)
    print 'X',X
    print 'y',y

    pos_x_names = ["Close", "Far"]
    pos_y_names = ["Wall", "Center"]
    ObjectName = ["crayola", "oreo", "book", "bottle", "clothes"]
    br = tf.TransformBroadcaster()
    rospy.sleep(1.5)
    num_bin = 0
    GoalInputObj = {}
    GoalInputPos = {}
    for i in range(num_objects):
        #if len(X[i].shape)==0 or X[i].shape[0]==0:
        #    print 'Object %s has 0 measures' % (i)
        #else:
        #    print 'Object %s has %s measures' % (i, X[i].shape[0])
        if len(X[i].shape)==0 or X[i].shape[0]<threshold_per_object:
            #Decide which quaternion
            maximum_needed = 0
            quat_to_do = 0
            pos_x_to_do = 0
            pos_y_to_do = 0
            for pos_x in range(2):
                for pos_y in range(2):
                    for j in range(num_goal_quaternions):
                        count = 0
                        for k in range(X[i].shape[0]):
                            count = count + max(0, 1-OneObjectDistance(pos_x,pos_y,X[i][k],NB[i][k],GoalQuaternions[j]))
                        if GoalMeasures[pos_x][pos_y][j]-count > maximum_needed:
                            maximum_needed = GoalMeasures[pos_x][pos_y][j] - count
                            quat_to_do = j
                            pos_x_to_do = pos_x
                            pos_y_to_do = pos_y

            if maximum_needed == 0:
                print '[Generate Scenes] Warning: even though object wants more samples, we dont need specific quaternions'
            #TODO: publish more time
            #TODO: visualize also position
            bin_position = np.zeros(3)
            posxy = [num_bin%2, 1, int(num_bin/2)%2]
            pos_x_to_do_str = 'Left'
            pos_y_to_do_str = 'Front'
            if pos_x_to_do==1:
                pos_x_to_do_str = 'Right'
            if pos_y_to_do==1:
                pos_y_to_do_str = 'Back'
            print '%s: Bin %s %s %s' % (ObjectName[i],num_bin,pos_x_to_do_str,pos_y_to_do_str)
            for axis in range(3):
                if axis!=2:
                    bin_position[axis] = (get_bin_cnstr()[num_bin][2*axis]*(2-posxy[axis])+get_bin_cnstr()[num_bin][2*axis+1]*(1+posxy[axis]))/3.
                else:
                    bin_position[axis] = (get_bin_cnstr()[num_bin][2*axis]+get_bin_cnstr()[num_bin][2*axis+1])/2.
                #bin_position[axis] = (get_bin_cnstr()[num_bin][2*axis]+get_bin_cnstr()[num_bin][2*axis+1])/2.

            #print 'Hello %s , %s, %s, %s' % (i, posxy, num_bin, bin_position)
            br.sendTransform(tuple(bin_position),
                             tuple(GoalQuaternions[num_bin]), #quat_to_do
                             rospy.Time.now(),
                             "input_bin_%s" % num_bin,'/shelf'
                             )
            #rospy.Timer(rospy.Duration(0.01), frameCallbackShelf(
            #    bin_position = bin_position,
            #    quaternion = GoalQuaternions[num_bin],
            #    num_bin = num_bin))
            num_bin = num_bin + 1
            GoalInputObj[num_bin] = i
            GoalInputPos[num_bin] = [pos_x_to_do, pos_y_to_do, GoalQuaternions[num_bin]]
        else:
            print '%s:' % (ObjectName[i])
    rospy.sleep(1.5)
    #response = 'bad'
    #while response!='' and response!='Y' and response!='N' and response!='n' and response!='Y':
    #   response = input('Would you like to change any bin?[y/N]')
    #   if response=='y' or response=='Y':
    #       bin_problem = input('Which bin?[y/N]')
    #       print 'I am going to assume quaternion is physically impossible, if thats not your problem email Ferran'

    #TODO: introduce a way for the human to tell the system that
    #a specific quaternion is not physically possible.
    return [GoalInputObj,GoalInputPos]

def OneObjectGP(bag):
    for obj in range(1):
        [X,y] = BagToArrays(bag,3,7)
        # Instanciate and fit Gaussian Process Model
        gp = GaussianProcess(theta0=5e-1)
        # Don't perform MLE or you'll get a perfect prediction for this simple example!
        print type(X[obj])
        print X[obj].shape
        for j in range((X[obj].shape)[0]):
            print j,type(X[obj][j]),X[obj][j].shape
        print type(y[obj])
        print X[obj]
        print y[obj]
        print X[obj].shape
        print y[obj].shape
        gp.fit(X[obj], y[obj])
        #TODO: add circular kernel
        #PlotGP(gp,X[obj],y[obj])

def EvaluateGrasping(features):
    name_feat = ['bias','dist_left','dist_right','clutter_front','clutter_top']
    coefs = [-1.,+20.,+20.,-2.,-2.]
    suma = 0.
    print '%s' % ('-'*70)
    for i in range(5):
        print '%s: %s * %s = %s' % (name_feat[i],coefs[i],features[name_feat[i]],features[name_feat[i]]*coefs[i])
        suma = suma + features[name_feat[i]]*coefs[i]
    return 1/(1+math.exp(-suma))

def EvaluateFrontSuction(features):
    name_feat = ['bias','front','clutter_front','clutter_top']
    coefs = [6.5,-2.,-4.,-5.]
    suma = 0.
    print '%s' % ('-'*70)
    for i in range(4):
        print '%s: %s * %s = %s' % (name_feat[i],coefs[i],features[name_feat[i]],features[name_feat[i]]*coefs[i])
        suma = suma + features[name_feat[i]]*coefs[i]
    return 1/(1+math.exp(-suma))

def EvaluateTopSuction(features):
    name_feat = ['bias','front','clutter_front','clutter_top']
    coefs = [6.,-1.,-1.,-8.]
    suma = 0.
    print '%s' % ('-'*70)
    for i in range(4):
        print '%s: %s * %s = %s' % (name_feat[i],coefs[i],features[name_feat[i]],features[name_feat[i]]*coefs[i])
        suma = suma + features[name_feat[i]]*coefs[i]
    return 1/(1+math.exp(-suma))


def PrototypeSmarty(scene):
    one_move_features = Featurize(scene)
    print '[Alpha-Cube] Trying only one move'
    best_one = 'threshold'
    threshold = 0.95
    best = threshold
    prob_grasping = EvaluateGrasping(one_move_features)
    prob_front_suction = EvaluateFrontSuction(one_move_features)
    prob_top_suction = EvaluateTopSuction(one_move_features)
    print 'Probability of successful grasping: %s' % prob_grasping
    if prob_grasping>best:
        best_one = 'grasping'
        bets = prob_grasping
    print 'Probability of successful FrontSuction: %s' % prob_front_suction
    if prob_front_suction>best:
        best_one = 'front_suction'
        best = prob_front_suction
    print 'Probability of successful TopSuction: %s' % prob_top_suction
    if prob_top_suction>best:
        best_one = 'top_suction'
        best = prob_top_suction
    if best_one=='threshold':
        print '[Alpha-Cube] No primitive with probability higher than threshold %s' % threshold
    else:
        print '[Alpha-Cube] Primitive %s will execute!' % best_one
        return
    options = np.array(['grasping', 'front_suction', 'top_suction'])
    for i in range(scene.num_objects):
        if i==scene.goal_object:
            continue
        real_goal = scene.goal_object
        scene.goal_object = i
        feat_t_i = Featurize(scene)
        probs_i = np.array([EvaluateGrasping(feat_t_i),EvaluateFrontSuction(feat_t_i),EvaluateTopSuction(feat_t_i)])
        scene.goal_object = real_goal
        print 'Probability of removing object %s: %s' % (i, np.amax(probs_i))
        if np.amax(probs_i) < threshold:
            continue
        position_i = copy.deepcopy(scene.objects[i].position)
        scene.objects[i].position = [-10, -10, -10, 0,0,0,1] #Out of the scene
        feat_m_i = Featurize(scene) #Features taking out iminus i.
        probs_m_i = np.array([EvaluateGrasping(feat_m_i),EvaluateFrontSuction(feat_m_i),EvaluateTopSuction(feat_m_i)])
        prim_m_i = np.argmax(probs_m_i)
        print 'Probability of 2-step removing object %s: %s * %s = %s' % (i, np.amax(probs_i) , probs_m_i[prim_m_i], np.amax(probs_i)*probs_m_i[prim_m_i])
        if np.amax(probs_i)*probs_m_i[prim_m_i] > best:
            best = np.amax(probs_i) * probs_m_i[prim_m_i]
            best_one = 'Remove object %s using %s and then use %s' % (i, options[np.argmax(probs_i)], options[prim_m_i])

        scene.objects[i].position = position_i #Back!
    print '[Alpha-Cube] %s with probability %s' % (best_one, best)

def main(argv=None):
    print 'a'
    rospy.init_node('analyze_fake_data', anonymous=False)
    dir_bagfile = os.environ['APCDATA_BASE']+'/learning/attempt1'
    make_sure_path_exists(dir_bagfile)
    bagfilename = 'single_object.bag'
    bag = rosbag.Bag(dir_bagfile+'/'+bagfilename, 'r')
    print bag
    #topics = ['fake_xy']
    #rosbag_proc = start_ros_bag(bagfilename, topics, dir_save_bagfile)
    count = 0
    rospy.sleep(5.)
    for topic, msg, t in bag.read_messages(topics=['/Ferran/fake_xy']):
        print msg.x,msg.y,msg.x+msg.y
        count = count+1
    for topic, msg, t in bag.read_messages(topics=['/Ferran/objects']):
        print msg.ObjId,msg.binId
        print msg.position
        count = count+1
    lastscene = APC_Scenario()
    for topic, msg, t in bag.read_messages(topics=['/Ferran/scenes']):
        print 'Header: ', msg.header
        print 'Objects: ', msg.objects
        print 'Goal Object: ', msg.goal_object
        print msg.primitive
        print msg.result
        lastscene = msg
        if msg.num_objects>1:
            features = Featurize(msg)
            print features
            #print 'Probability of successful grasping: %s' % EvaluateGrasping(features)
            #print 'Probability of successful FrontSuction: %s' % EvaluateFrontSuction(features)
            #print 'Probability of successful TopSuction: %s' % EvaluateTopSuction(features)
            PrototypeSmarty(msg)
            print '%s' % ('-'*70)
        else:
            print 'Cannot featurize because only 1 object'
        count = count+1
    OneObjectSceneGeneration(bag)
    #OneObjectGP(bag)
    print 'Finished analayzing ', count
    rospy.sleep(1.)

    #print 'Will try to add elements to rosbag'

    #bag.write('/Ferran/scenes', msg)
    #rospy.sleep(1.)
    bag.close()
    #pub = rospy.Publisher('fake_xy', fake_struct, queue_size=100)
    #terminate_ros_node("/record")

if __name__=='__main__': # What does this do?
    main()
    #    sys.exit(main())
