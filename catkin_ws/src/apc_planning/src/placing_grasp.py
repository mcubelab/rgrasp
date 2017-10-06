#!/usr/bin/python
import rospy
from ik.helper import get_params_yaml, rotmatZ, mat2quat
import sys
import numpy as np
import math
from math import ceil, floor
import optparse
import copy
import matplotlib.image as img
from sensor_msgs.msg import Image as RosImage
import matplotlib
from PIL import Image
import os
import time
import json
from ik.helper import Timer
try:
    import passive_vision.srv
    import realsense_camera.srv
except:
    print 'FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL'

class PlacingPlanner(object):
    def __init__(self, e = None, visionType = 'virtual'):
        self.visionType = visionType #virtual or real
        self.task = 'stowing'
        self.combined_task = True
        self.time_since_clean = [floor(time.time())]*4
        self.num_bins = 3+5 #number of bins and boxes
        self.available = [True, True, True, True, True, True,True, True]
        self.box_placing = False
        self.disc = 0.01
        self.vis_disc = 0.002
        self.tote_length = rospy.get_param("/tote/length")
        self.tote_width = rospy.get_param("/tote/width")
        self.tote_height = rospy.get_param("/tote/height")
        self.limit_x = 8
        self.limit_y = 4 #Manually adjusted
        self.h = [self.tote_height for _ in range(self.num_bins)]
        # Two of the bins have half the heigh
        self.h[1]/=2
        self.h[2]/=2
        self.h[3] = rospy.get_param("/bin4/height"); self.h[4] = rospy.get_param("/bin5/height"); self.h[5] = rospy.get_param("/bin6/height")
        self.h[6] = rospy.get_param("/bin7/height"); self.h[7] = rospy.get_param("/bin8/height")
        self.h_max = 0
        self.d = [[self.tote_length, self.tote_width] for _ in range(self.num_bins)]
        self.discDim = [[int(math.floor(x[0]/self.disc)), int(math.floor(x[1]/self.disc))] for x in self.d] #discretized dimensions of the HeightMap
        
        self.HeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Height Maps
        self.VarHeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Var Height Maps
        self.ExpHeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Exp Height Maps
        self.Scores = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Scores Maps
        self.Scores_rotated = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Scores Maps
        self.OldHeightMap = [np.zeros([self.discDim[i][0], self.discDim[i][1]]) for i in
                range(self.num_bins)] #Scores Map
                                
        self.objects = []
        self.n = 10 #number of objects
        self.toteObj = [] #List of indices of objects left
        self.binObj = [[] for _ in range(self.num_bins)] #TODO: uses num_bins but does not seem to be used

        self.last_x_taken = 0
        self.last_y_taken = 0

        if self.visionType == 'real':
            self.getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)

        #Cost function
        self.coef_similarity = 1.
        self.coef_obj_on_top_others = 1.

        #Parameters
        self.pow_variance = 4.
        self.INF = 100000.
        
        #High_surface_objects
        self.list_high_surface = ['avery_binder','composition_book', 'table_cloth',
            'hanes_socks','ice_cube_tray','pie_plates'] 
            #burts_bees_baby_wipes, epsom_salts,robots_everywhere, windex

    #####################
    ### Visualization ###
    #####################
    def ros_sensor_image_from_matrix(self, mat, normalize = False):
        image_message = RosImage()
        image_message.header.stamp = rospy.Time.now()
        height = mat.ravel().astype(np.uint8).tolist()
        image_message.data = []
        if normalize:
            normalizer = matplotlib.colors.Normalize()
            normalizer.autoscale(height)
        for el in height:
            if normalize:
                image_message.data += np.array([(255 * (1 - normalizer(el))), 0, (255 * normalizer(el))]).astype(np.uint8).tolist()
            else:
                image_message.data += [255 - el, 0, el]

        image_message.height = len(mat)
        image_message.width = len(mat[0])
        image_message.is_bigendian = 0
        image_message.encoding = 'bgr8'
        return image_message

    def show_placing_decision(self, hm_pre_pub, hm_post_pub, score_pub, b = 0, theta = 0):
        '''Based on height map, generate a matplotlib plot
        to show the heights. Either save or display.
        @param mapNumber the bin of the heightmap to display, None
                   will loop through all bins
        @param saveFile (bool) Whether to save plot to file
        '''
        HM = np.multiply(self.HeightMap[b], 100)
        old_HM = np.multiply(self.OldHeightMap[b], 100)
        import copy
        if theta == 0:
            Scores = copy.deepcopy(self.Scores[b])
        else:
            Scores = copy.deepcopy(self.Scores_rotated[b])
        max_score_lower_INF = 0
        for x in range(len(HM)):
            for y in range(len(HM[0])):
                if Scores[x][y] < self.INF:
                    max_score_lower_INF = max(max_score_lower_INF, Scores[x][y])
        for x in range(len(HM)):
            for y in range(len(HM[0])):
                if Scores[x][y] >= self.INF:
                    Scores[x][y] = max_score_lower_INF
        (y_dim, x_dim) = HM.shape
        '''
        #Save HeightMaps
        self.HMfilename = os.environ['CODE_BASE']+'/output/height_map_result_in_bin%d.json'%(b+1)
        DATA = {}
        DATA['old_HM'] = old_HM.tolist()
        DATA['Scores'] = Scores.tolist()
        DATA['HM'] = HM.tolist()
        with open(self.HMfilename, 'w') as outfile:
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        with open(self.HMfilename + '.' + str(rospy.get_time()), 'w') as outfile:   
            json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
        
        '''
        # Plot OldHM
        hm_pre_pub.publish(self.ros_sensor_image_from_matrix(np.multiply(old_HM, 20)))
        # Plot scores
        score_pub.publish(self.ros_sensor_image_from_matrix(np.multiply(Scores, 20), normalize= True))
        # Plot new HM
        hm_post_pub.publish(self.ros_sensor_image_from_matrix(np.multiply(HM, 20)))
        lastNumber = -1
        for y in xrange(y_dim):
            for x in xrange(x_dim):
                if lastNumber != old_HM[y][x]:
                    lastNumber = old_HM[y][x]
        lastNumber = -1
        for y in xrange(y_dim):
            for x in xrange(x_dim):
                if lastNumber != HM[y][x]:
                    lastNumber = HM[y][x]
                    
        #raise Exception('asdf')
    
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

    def compute_variance_height_maps(self, HeightMap=0):
        if type(HeightMap) == int:
            in_self = True
            b = HeightMap
            HeightMap = self.HeightMap[b] #Remember this is a POINTER copy
            VarHeightMap = self.VarHeightMap[b]
            ExpHeightMap = self.ExpHeightMap[b]
        else:
            in_self = False
            VarHeightMap = copy.copy(HeightMap)
            ExpHeightMap = copy.copy(HeightMap)
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
        if not in_self:
            return VarHeightMap

    def get_local_scores(self, obj, HeightMap=0, b=0):
        
        #Try to make it fast
        self.best_local_score = self.INF*10
        best_x = -1
        best_y = -1
        if type(HeightMap) == int:
            in_self = True
            b = HeightMap
            HeightMap = self.HeightMap[b]
            VarHeightMap = self.VarHeightMap[b]
            ExpHeightMap = self.ExpHeightMap[b]
        else:
            in_self = False
        #TODO: stability is % of points close to max height
        if in_self and not self.available[b]:  #If bin is not available, assign high scores
            #print('Bin ',b+1,' not available')
            Scores = 5.*self.INF*np.ones([len(HeightMap), len(HeightMap[0])])
            return Scores, [self.best_local_score, b, best_x, best_y, 1000]
        Scores = np.zeros([len(HeightMap), len(HeightMap[0])])
        max_h = 100
        min_max_h = 100
        '''
        for x in range(len(HeightMap)):
            for y in range(len(HeightMap[0])):
                #Check if in the cell (x,y) the object center would be outside the tote:
                if x < self.limit_x or y< self.limit_y or x >= len(HeightMap)-self.limit_x or y >= len(HeightMap[0])-self.limit_y:
                    Scores[x][y] = 4*self.INF
                    continue
                p = [x*self.disc, y*self.disc]
                #Check if the object fits there
                [[mx, Mx], [my, My]] = [[int(math.floor((p[_]-obj.w_dim[_]/2.)/self.disc)), int(math.ceil((p[_]+obj.w_dim[_]/2.)/self.disc))] for _ in range(2)]
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
                #Check what type of object are we considering during the combined task and in the bin 1(0)
                if b == 0 and self.combined_task:
                    bin_separation = len(HeightMap)/2
                    if Mx > bin_separation and (obj.label in self.list_high_surface):
                        Scores[x][y]  += self.INF/10. #self.INF = 100000.
                        #print 'deciding_side_small_x'
                    if mx <= bin_separation and (obj.label not in self.list_high_surface):
                        Scores[x][y]  += self.INF/10.
                        #print 'deciding_side_big_x'
                # Get maximum and minum height in the place where the object will be placed
                max_h = self.get_minmax_HM(False, [mx, my], [Mx, My], HeightMap)
                min_h = self.get_minmax_HM(True, [mx, my], [Mx, My], HeightMap)
                #print b,x,y,mx,Mx,my,My, obj.w_dim, obj.label, min_h, max_h
                min_max_h = min(max_h, min_max_h)
                if max_h + obj.w_dim[2] > self.h[b]:
                    Scores[x][y] = self.INF + max_h*self.INF/10.
                else:
                    if max_h < 0.02:  #TODO: this seems pretty wrong
                        Scores[x][y] += (self.h[b]-obj.w_dim[2])/self.disc #small in small big in big
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
                #print(var=var)
                var = pow(var, 1./self.pow_variance)
                Scores[x][y] += var/self.disc
                Scores[x][y] += (max_h-min_h)/self.disc
                
                if (self.task == 'picking' or self.combined_task == True) and b < 3: #we are considering placing in a bin, not a box
                    for i, obj2 in enumerate(self.objects):
                        if obj2.container == b+1 and obj2 != obj and (obj.is_goal == 'yes' or obj2.is_goal == 'yes' or obj.is_goal == 'maybe' or obj2.is_goal == 'maybe'):  #If we are considering the same container, different objec and one of them is goal
                            #Check if weights are close. If two objects weight the same, we assume a difference of only 1g
                            weight_dist = max(0.001, abs(obj.weight - obj2.weight)) 
                            Scores[x][y] += 1./(weight_dist) #show values to make sure is reasonable
                #Prefer bin 2 and 3 rather than bin 1 when doing stowing in the combined task
                if self.task == 'picking' and self.combined_task == True and b == 2: 
                    Scores[x][y] += 15
                if Scores[x][y] < self.best_local_score:
                    self.best_local_score = copy.deepcopy(Scores[x][y])
                    best_x = copy.deepcopy(x)
                    best_y = copy.deepcopy(y)
        
        print('object dims', obj.w_dim)
        print(dims=obj.w_dim)
        print(b=b)
        print(T=type(Scores))
        '''
        if True: #best_x == -1:
            #print('The object dimensions: ', obj.w_dim, ' are probably very large Placing in the center of bin:', b+1)
            best_x = copy.deepcopy(floor(len(HeightMap)/2))
            best_y = copy.deepcopy(floor(len(HeightMap[0])/2))
            self.best_local_score = copy.deepcopy(self.INF*10)
        objprint = 0 #TODO_M
        if objprint == 0:
            self.Scores[b] = Scores #Keep it
        else:
            self.Scores_rotated[b] = Scores #Keep it
        print('best score: ',self.best_local_score, ' in bin: ', b+1, ' is at (x,y): ', best_x, best_y)
        return Scores, [self.best_local_score, b, best_x, best_y, max_h]

    def get_best_local_positions(self, obj, HeightMap=0, b=0, k=5):
        if type(HeightMap) == int:
            b = HeightMap
        candidates = []
        Scores, candidate = self.get_local_scores(obj, HeightMap, b)
        candidates.append(candidate)
        return candidates #TODO: only 1 option by now
        '''
        candidates = []
        for x in range(len(Scores)):
            for y in range(len(Scores[x])):
                if Scores[x][y] == self.best_local_score: 
                    candidates.append([Scores[x][y], b, x, y, objprint, max_h]) #TODO: max_h
        candidates.sort()
        return candidates[:k] #Return 5 elements altough only use first one
        '''

    def place_object_local_best(self, obj=0, containers = None, undesired_containers = []):
        if type(obj) == int:
            obj = self.objects[obj]
        #print(OBJECT=obj.label)
        if containers is None:
            containers = range(3)
        #Candidates given inital orientation
        #objprint = 0
        cand = [self.get_best_local_positions(obj = obj, HeightMap = int(i)) for i in containers]
        #Candidates given 90 degrees change in orientation
        #objprint = np.pi/2
        #obj.w_dim[0], obj.w_dim[1] = obj.w_dim[1], obj.w_dim[0] #Swap x,y dimensions
        #cand_rot = [self.get_best_local_positions(obj = obj, HeightMap = int(i)) for i in containers]
        
            
        #obj.w_dim[0], obj.w_dim[1] = obj.w_dim[1], obj.w_dim[0] #Swap x,y dimensions
        
        #Merge candidates
        C = []
        for c in cand:
            if c[0][1] in undesired_containers:
                c[0][0] += self.INF/2.
            C += c
        '''
        for c in cand_rot:
            if c[0][1] in undesired_containers:
                c[0][0] += self.INF/2.
            C += c
        '''
        C.sort()
        #Get best bin, position and orientation
        b = C[0][1]
        [x, y] = C[0][2:4]
        #obj.container = b + 1
        #obj.pos = [x*self.disc, y*self.disc]
        #objprint = C[0][4]
        #self.h_max = C[0][4]
        '''
        print(Limit_x = self.limit_x)
        print(Limit_y = self.limit_y)
        print(x = x)
        print(y = y)
        print(AssignedPos=obj.pos)
        print(AssignedTheta=objprint)
        print(AssignedScore=C[0][0])
        '''
        
        self.last_x_taken = x
        self.last_y_taken = y
        return C[0][0]
    
    def place_object_choosebin(self, obj=0, containers = None):
        # SS: choose the best bin base on size and weight
        if type(obj) == int:
            obj = self.objects[obj]

        if containers is None:
            containers = range(3)

        # numObjPerBin = np.zeros([len(containers)])
        # for i, obj2 in enumerate(self.objects):
        #     if obj2 != obj and obj2.container > 0:
        #         numObjPerBin[obj2.container-1] += 1



        #SS: weight score only consider the nearest neighbour 
        weightScore = np.zeros([len(containers)]) #initialize with zeros, empty bin should have a good score 
        for i, obj2 in enumerate(self.objects):
            if obj2 != obj and obj2.container > 0:
                if (obj2.label == obj.label): # put same object in same bin
                    return obj2.container
                    # weightScore = np.ones([len(containers)])*self.Inf
                    # weightScore[obj2.container-1] = 0
                    # break
                else:
                    weight_dist = max(0.001, abs(obj.weight - obj2.weight))
                    weightScore[obj2.container-1] = max(weightScore[obj2.container-1],1./(weight_dist))

        # #Empty bin should have a good score 
        # for i in range(len(containers)):
        #     if numObjPerBin[i] == 0:
        #         weightScore[i] = 0

        #SS: check whether size can fit 
        if self.h[2] > min(obj.w_dim):
            weightScore[0] = self.INF # fits in small bins
        else:
            weightScore[0] = 0 # cannot fits in small bins go to the first bin
            
        best_binid = np.argmin(weightScore)+1

        #show values to make sure is reasonable
        print('weightScore: ',weightScore)

        return best_binid

    ####################
    ### Modify state ###
    ####################

    def put_objects_from_planner(self, objects):
        self.objects = objects #I think this is a pointer, would be great if it is
        self.n = len(self.objects)

    def put_random_objects(self, n, low_d = [0.07, 0.03,0.02], high_d = [0.20,0.20,0.13]): #If used, only in main
        self.n = n
        for i in range(n):
            self.objects.append(Item())
            self.objects[-1].update_with_label('box'+str(i))
            self.objects[-1].dim = [np.random.uniform(low_d[_], high_d[_]) for _ in range(3)]
            self.toteObjects = range(n)

    def put_objects_from_input(self): #Only used in main
        self.n = '-1'
        ok = False
        while not ok:
            self.n = raw_input('How many objects do you want to place?')
            ok = True
            try:
                self.n = int(self.n)
            except:
                ok = False
            if self.n <= 0:
                print('Too few objects')
                ok = False
            elif self.n > 50:
                print('Too many objects')
                ok = False
        self.objects = []
        for n_o in range(self.n):
            self.objects.append(Item())
            ok = self.objects[-1].update_with_label(raw_input('Pass object name: '))
            if not ok:
                ok = self.objects[-1].update_with_label(raw_input('Label seems not ok, try again: '))


    def evaluate_final_scene(self): #gives an evaluation on the objects on the shelf
        scene_cost = 0
        #Penalty for similar objects nearby
        for i in range(n):
            for j in range(i, n):
                #Two objects are confused if similar weight and similar position
                if self.Objects[i].container != self.Objects[j].container:
                    continue
                dist = np.norm(self.Objects[i].pos[:2]-self.Objects[j].pos[:2])
                prob_same_pos = max(0., min(1., 1.25-dist/20.)) #10:0.75, 15.: 0.5, 20: 0.25
                prob_same_weight = 1. #Some gaussian
                scene_cost += self.coef_sim *prob_same_pos*prob_same_weight
        return scene_cost

    def update_real_height_map(self, b = None, img_path = None):
        if b == None:
            for i in range(3):  #There are only 3 bins and 3 boxes
                self.update_real_height_map(i)
            return
        else:
            assert type(b) == int, 'Can only update HM if b=None or an integer'
            HM = self.HeightMap[b]  #this is pointer cop so self.HeightMap[b] will be updated when modifying HM
        # Get height map from vision
        #update foreground-top-view
        if self.available[b]:
            # self.passive_vision_state = self.getPassiveVisionEstimate('',b+1) # change
            # while long(self.passive_vision_state.heightmap_timestamp) < self.time_since_clean[b+1]:
                # print('Vision is not yet updated, clean time according planner: ', self.time_since_clean[b+1], ' according vision: ', self.passive_vision_state.heightmap_timestamp)
                # time.sleep(0.5)
                # self.passive_vision_state = self.getPassiveVisionEstimate('', b+1) #self.get_estimate('', bin_id)
            with Timer('Call passive vision to request new height map for %d' % (b+1)):
                print('getPassiveVisionEstimate ', 'request', '',  b+1)
                
                while True:
                    try:
                        self.passive_vision_state = self.getPassiveVisionEstimate('request', '',  b+1)
                        break
                    except:
                        e.db('Keep waiting.')
            M = np.asarray(self.passive_vision_state.height_map)
            if len(M) > 0:
                M = M.reshape((200,300))
                M = M.transpose()
            else:
                print('There is no HM, we will be using the old one....')
                return
        else:
            file_name =os.environ['ARCDATA_BASE']+'/graspdata/debug/foreground-top-view.depth.png'
            M = img.imread(file_name)
            M = np.transpose(M)
        #import ipdb; ipdb.set_trace()

        for i in range(len(HM)-2):
            for j in range(len(HM[i])-2):
                [x_ini, x_fin, y_ini, y_fin] = [int(floor(i*self.disc/self.vis_disc)), int(ceil((i+1)*self.disc/self.vis_disc)), int(floor(j*self.disc/self.vis_disc)), int(ceil((j+1)*self.disc/self.vis_disc))]
                x_fin = max(x_ini+1, x_fin)
                y_fin = max(y_ini+1, y_fin)
                #print i,',',j,': ',len(HM),',',len(HM[0]),':',x_ini, x_fin, y_ini, y_fin, len(M), len(M[0])
                HM[i][j] = np.max(M[[x_ini,x_fin],:][:,[y_ini,y_fin]])
        
        #Now that you have the right HM, update other maps. 
        self.compute_variance_height_maps(b)

        
    def add_object_to_height_map(self, obj, HeightMap=0): #Only used in virtual mode for vision.
        #If HeightMap=int, take it as a bin index
        #print(LABEL=obj.label)
        if type(HeightMap) == int:
            in_self = True
            b = HeightMap
        else:
            in_self = False
        import copy
        self.OldHeightMap = copy.deepcopy(self.HeightMap)
        #print 'obj.pos, obj.w_dim = ', obj.pos, obj.w_dim
        #if objprint > 0:
        #    obj.w_dim[0], obj.w_dim[1] = obj.w_dim[1], obj.w_dim[0]
        limits = [[int(math.floor((obj.pos[_]-obj.w_dim[_]/2.)/self.disc)), int(math.ceil((obj.pos[_]+obj.w_dim[_]/2.)/self.disc))] for _ in range(2)]
        '''
        print(limits=limits)
        print(w_dim=obj.w_dim)
        print(pos=obj.pos)
        print(b=b)
        '''
        new_height = self.get_minmax_HM(compute_min=False, min_lim=[limits[0][0], limits[1][0]], max_lim=[limits[0][1], limits[1][1]], HeightMap=HeightMap)
        #print(new_height=new_height)
        
        new_height += obj.w_dim[2]
        #print(limits)
        min_x = max(0, limits[0][0])
        max_x = min(len(self.HeightMap[b]), limits[0][1]+1)
        min_y = max(0, limits[1][0])
        max_y = min(len(self.HeightMap[b][0]), limits[1][1]+1)
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                if in_self:
                    self.HeightMap[b][x][y] = new_height
                else:
                    HeightMap[x][y] = new_height
        if type(HeightMap) == int:
            self.compute_variance_height_maps(b)

    def update_pose_placed_obj(self, obj):
        
        if type(obj) == int:
            obj = self.objects[obj]
        #Add to the HM
        self.add_object_to_height_map(obj, int(obj.container-1))
        obj.pose[0:3] = [obj.pos[0]-self.tote_length/2., obj.pos[1]-self.tote_width/2.,obj.w_dim[2]/2+self.h_max] #Put pos with respect to the center of the tote
        center_bin = get_params_yaml('bin'+str(obj.container)+'_pose')[0:3]  #TODO_M: there is a relation between boxes names and identity that is not related to our 4,5,6
        obj.pose[0:3] = [obj.pose[_] + center_bin[_] for _ in range(3)]
        #convert objprint into quaternion:
        orient_mat_3x3 = rotmatZ(0*180.0/np.pi)  #rotmatZ(objprint*180.0/np.pi) 
        obj.pose[3:7] = mat2quat(orient_mat_3x3)
        print('Added object:', obj.label,' to HM considering obj.pos: ', obj.pos, ', bin center: ', center_bin, 'and obj.pose: ', obj.pose, ' in obj.container: ', obj.container)
        return obj

def main(): #Does a small demo of the planner
    parser = optparse.OptionParser()
    (opt, args) = parser.parse_args()
    P = PlacingPlanner(args)
    P.put_objects_from_input()
    #P.put_random_objects(32)
    for i in range(P.n):
        P.objects[i].random_w_dim()
        P.e.db('Placing objects of dimensions', P.objects[i].w_dim)
        #aux = raw_input('continue')
        P.e.db(PassedObject=P.objects[i].label)
        #aux = raw_input('continue')
        P.place_object_local_best(P.objects[i])
        #if i % 1 == 0:
            #aux = raw_input('continue')
    '''
    P.objects[1].w_dim = copy.copy(P.objects[1].dim)
    P.objects[1].pos = [10,10,0]
    P.add_object_to_height_map(P.objects[1])
    '''

if __name__ == "__main__":
    main()
