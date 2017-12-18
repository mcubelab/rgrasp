#!/usr/bin/python
import rospy
from ik.helper import get_params_yaml, rotmatZ, mat2quat
import numpy as np
import math
from math import ceil, floor
import optparse
import copy
import matplotlib.image as img
import sensor_msgs.msg
from sensor_msgs.msg import Image as RosImage
import matplotlib
import os
import time
from ik.helper import Timer
try:
    import passive_vision.srv
except:
    print('FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL')

class PlacingPlanner(object):
    def __init__(self, e = None, visionType = 'virtual'):
        self.visionType = visionType #virtual or real
        self.num_bins = 3
        self.available = [True, True, True]
        self.box_placing = False
        self.disc = 0.01
        self.vis_disc = 0.002
        self.tote_length = rospy.get_param("/tote/length")
        self.tote_width = rospy.get_param("/tote/width")
        self.tote_height = rospy.get_param("/tote/height")
        self.limit_x = 8 #Manually adjusted
        self.limit_y = 4 #Manually adjusted
        self.h = [self.tote_height for _ in range(self.num_bins)]
        self.h_max = 0
        self.d = [[self.tote_length, self.tote_width] for _ in range(self.num_bins)]
        self.discDim = [[int(math.floor(x[0]/self.disc)), int(math.floor(x[1]/self.disc))] for x in self.d] #discretized dimensions of the HeightMap
        
        self.HeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Height Maps
        self.VarHeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Var Height Maps
        self.ExpHeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Exp Height Maps
        self.OldHeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Scores Map

        if self.visionType == 'real':
            self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)

        #Parameters
        self.pow_variance = 4.
        self.INF = 100000.


    #############################
    ### Computing Statistics ####
    #############################
    def get_minmax_HM(self, compute_min, min_lim=[0, 0], max_lim=None, HeightMap=0):
        ## min_lim and max_lim give an idea of where are the corners of the object if placed in a certain position in the tote. They are given in the tote frame (in on of its corners)
        b = 0 #Default value...
        if type(HeightMap) == int:
            b = HeightMap
            in_self = True
        else:
            in_self = False
            b = 0
        if max_lim == None:
            max_lim = [self.disc_d[0], self.disc_d[1]]
        res = -10
        if compute_min:
            res = 1000  #CHECK: this is cm's right? so that would be 1000cm = 10m
        if len(self.HeightMap[b]) == 0:
            return res
        min_x = max(0, min_lim[0])
        max_x = min(len(self.HeightMap[b]), max_lim[0]+1)
        min_y = max(0, min_lim[1])
        max_y = min(len(self.HeightMap[b][0]), max_lim[1]+1)
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                if compute_min:
                    if in_self:
                        res = min(res, self.HeightMap[b][x][y])
                        
                    else:
                        res = min(res, HeightMap[x][y])
                else:
                    if in_self:
                        res = max(res, self.HeightMap[b][x][y])
                    else:
                        res = max(res, HeightMap[x][y])
        assert res > -10 and res < 1000, 'WTF: get_minmax_HM is wrong'
        return res

    def get_local_scores(self, obj_dim=[0.12,0.12,0.12], b=0):
        
        self.best_local_score = self.INF*10
        best_x = -1
        best_y = -1    
        HeightMap = self.HeightMap[b]
        VarHeightMap = self.VarHeightMap[b]
        ExpHeightMap = self.ExpHeightMap[b]

        if not self.available[b]:  #If bin is not available, assign high scores
            Scores = 5.*self.INF*np.ones([len(HeightMap), len(HeightMap[0])])
            return Scores, [self.best_local_score, b, best_x, best_y, 1000]
        Scores = np.zeros([len(HeightMap), len(HeightMap[0])])
        max_h = 100
        min_max_h = 100
        
        for x in range(len(HeightMap)):
            for y in range(len(HeightMap[0])):
                #Check if in the cell (x,y) the object center would be outside the tote:
                if x < self.limit_x or y< self.limit_y or x >= len(HeightMap)-self.limit_x or y >= len(HeightMap[0])-self.limit_y:
                    Scores[x][y] = 4*self.INF
                    continue
                p = [x*self.disc, y*self.disc]
                #Check if the object fits there
                [[mx, Mx], [my, My]] = [[int(math.floor((p[_]-obj_dim[_]/2.)/self.disc)), int(math.ceil((p[_]+obj_dim[_]/2.)/self.disc))] for _ in range(2)]
                if mx < self.limit_x or my < self.limit_y or Mx >= len(HeightMap)-self.limit_x or My >= len(HeightMap[0])-self.limit_y:
                    Scores[x][y] = 2*self.INF
                    continue
                ##################
                ## Height score ##
                ##################
                Scores[x][y] += 0.01*x
                Scores[x][y] += 0.01*y
                if Scores[x][y] > self.best_local_score:
                    continue
                # Get maximum and minum height in the place where the object will be placed
                max_h = self.get_minmax_HM(False, [mx, my], [Mx, My], HeightMap)
                min_h = self.get_minmax_HM(True, [mx, my], [Mx, My], HeightMap)
                min_max_h = min(max_h, min_max_h)
                if max_h + obj_dim[2] > self.h[b]:
                    Scores[x][y] = self.INF + max_h*self.INF/10.
                else:
                    if max_h < 0.02:  #TODO: this seems pretty wrong
                        Scores[x][y] += (self.h[b]-obj_dim[2])/self.disc #small in small big in big
                    else:
                        Scores[x][y] += self.h[b]/self.disc  #Always prefer low options
                if Scores[x][y] > self.best_local_score:
                    continue
                ##################
                # Variance score #
                ##################
                variance = VarHeightMap[Mx][My]
                expvar = ExpHeightMap[Mx][My]
                if my > 0:
                    variance -= VarHeightMap[Mx][my-1]
                    expvar -= ExpHeightMap[Mx][my-1]
                if mx > 0:
                    variance -= VarHeightMap[mx-1][My]
                    expvar -= ExpHeightMap[mx-1][My]
                if mx > 0 and my > 0:
                    variance += VarHeightMap[mx-1][my-1]
                    expvar += ExpHeightMap[mx-1][my-1]
                N = float((Mx-mx+1)*(My-my+1))
                var = max(0, variance/N-pow(expvar/N, self.pow_variance)) + pow(0.0001, self.pow_variance)
                var = pow(var, 1./self.pow_variance)
                Scores[x][y] += var/self.disc
                Scores[x][y] += (max_h-min_h)/self.disc
                
                if Scores[x][y] < self.best_local_score:
                    self.best_local_score = copy.deepcopy(Scores[x][y])
                    best_x = copy.deepcopy(x)
                    best_y = copy.deepcopy(y)
        
        print('object dims', obj_dim)

        if best_x == -1:
            print('The object dimensions: ', obj_dim, ' are probably very large Placing in the center of bin:', b)
            best_x = copy.deepcopy(floor(len(HeightMap)/2))
            best_y = copy.deepcopy(floor(len(HeightMap[0])/2))
            self.best_local_score = copy.deepcopy(self.INF*10)



        print('best score: ',self.best_local_score, ' in bin: ', b, ' is at (x,y): ', best_x, best_y)

        ''' Compute true position'''
        obj_pos = [best_x*self.disc, best_y*self.disc]
        obj_pose = [obj_pos[0]-self.tote_length/2., obj_pos[1]-self.tote_width/2.,obj_dim[2]/2+self.h_max] #Put pos with respect to the center of the tote
        center_bin = get_params_yaml('bin'+str(b)+'_pose')[0:3] 
        obj_pose[0:3] = [obj_pose[_] + center_bin[_] for _ in range(3)]
        #convert obj.theta = 0 into quaternion:
        orient_mat_3x3 = rotmatZ(0) 
        obj_pose[3:7] = mat2quat(orient_mat_3x3)
        return Scores, [obj_pose, self.best_local_score, b, best_x, best_y, max_h]

    def get_best_local_positions(self, obj_dim=[0.12,0.12,0.12], b=0):
        Scores, candidate = self.get_local_scores(obj_dim=obj_dim, b = b)
        return [candidate]

    def place_object_local_best(self, obj_dim=[0.12,0.12,0.12], containers = None):
        if containers is None:
            containers = range(3)
        #Candidates given inital orientation
        C = [self.get_best_local_positions(obj_dim = obj_dim, b = b) for b in containers]
        C.sort()
        #Return best position
        return C[0][0][0]

   
    def compute_variance_height_maps(self, b=0):
        HeightMap = self.HeightMap[b] #Remember this is a POINTER copy
        VarHeightMap = self.VarHeightMap[b]
        ExpHeightMap = self.ExpHeightMap[b]
        for x in range(len(HeightMap)):
            for y in range(len(HeightMap[x])):
                ExpHeightMap[x][y] = HeightMap[x][y]
                VarHeightMap[x][y] = pow(HeightMap[x][y], self.pow_variance)
                if x > 0:
                    VarHeightMap[x][y] += VarHeightMap[x-1][y]
                    ExpHeightMap[x][y] += ExpHeightMap[x-1][y]
                if y > 0:
                    VarHeightMap[x][y] += VarHeightMap[x][y-1]
                    ExpHeightMap[x][y] += ExpHeightMap[x][y-1]
                if x > 0 and y > 0:
                    VarHeightMap[x][y] -= VarHeightMap[x-1][y-1]
                    ExpHeightMap[x][y] -= ExpHeightMap[x-1][y-1]

    
    def update_real_height_map(self, b):
        
        HM = self.HeightMap[b]  #this is pointer cop so self.HeightMap[b] will be updated when modifying HM
        # Get height map from vision
        if self.available[b]:
            with Timer('Call passive vision to request new height map for %d' % (b)):
                print('getPassiveVisionEstimate ', 'request', '',  b)
                while True:
                    try:
                        self.passive_vision_state = self.getPassiveVisionEstimate('request', '',  b)
                        break
                    except:
                        print('Keep waiting.')
            M = np.asarray(self.passive_vision_state.height_map)
            if len(M) > 0:
                M = M.reshape((200,300))
                M = M.transpose()
            else:  
                print('There is no HM, we will be using the old one....')
                import pdb; pdb.set_trace()
                return
        else:
            file_name =os.environ['ARCDATA_BASE']+'/graspdata/debug/foreground-top-view.depth.png'
            M = img.imread(file_name)
            M = np.transpose(M)

        for i in range(len(HM)-2):
            for j in range(len(HM[i])-2):
                [x_ini, x_fin, y_ini, y_fin] = [int(floor(i*self.disc/self.vis_disc)), int(ceil((i+1)*self.disc/self.vis_disc)), int(floor(j*self.disc/self.vis_disc)), int(ceil((j+1)*self.disc/self.vis_disc))]
                x_fin = max(x_ini+1, x_fin)
                y_fin = max(y_ini+1, y_fin)
                HM[i][j] = np.max(M[[x_ini,x_fin],:][:,[y_ini,y_fin]])
        
        #Now that you have the right HM, update other maps. 
        self.compute_variance_height_maps(b)

    

if __name__ == "__main__":
    main()
