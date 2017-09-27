#!/usr/bin/python
import yaml
from ik.helper import get_params_yaml
import rospy
import numpy as np

class Item(object):
    #TODO: implement class Product
    def __init__(self, label='', pose=[0]*6, weight=-1, container=-1, is_goal = 'maybe'):
        '''Set constants and variables for the objects'''
        #CONSTANTS
        self.META_LABELS = ['unknown', 'dummy', '']

        #Variables
        self.label = label
        if self.label == 'dummy':
            self.set_dummy()
            return
        self.weight = weight
        self.pose = pose #pos in world frame
        self.container = container #0: tote, 1..3: bins, -1: unknown
        self.is_goal = is_goal #whether item has to go on a box[yes,no,maybe]
        self.dim = [-1, -1, -1]
        self.placeability = [1, 1, 1]
        self.w_dim = [-1, -1, -1] #World dimensions: dx,dy,dz and theta
        self.theta = -1 #angle of rotation
        self.pos = [-1, -1, -1] #position in bin/tote frame
        if self.label not in self.META_LABELS:
            self.update_with_label()

    def __str__(self):
        s = ''
        s += self.label + '[' + self.is_goal + ',' + str(self.container) + ']\t\t'
        #s += 'Label: ' + self.label + ', '
        #s += 'IsGoal: ' + self.is_goal + ', '
        #s += 'Container: ' + str(self.container) + ', '
        s += 'Dim: ' + str(self.dim) + ', '
        s += 'W-dim: ' + str(self.w_dim) + ', '
        return s

    def set_dummy(self):
        '''Set dummy initial values'''
        self.label = 'dummy'
        self.weight = '0.0'
        self.pose = [0]*6

    ##############################
    ### Modification functions ###
    ##############################
    def swap_container(self):
        '''Swap the container value from 0 to 3 or vice versa'''
        if self.container == 0:
            self.container = 3 #Assuming only one bin
        else:
            self.container = 0

    def random_w_dim(self):
        '''Set a random value for the w dimension'''
        self.w_dim = np.random.permutation(self.dim)

    def conservative_w_dim_from_dim(self):
        self.w_dim = [self.dim[0], self.dim[1], self.dim[2]]

    def update_w_dim_with_bbox(self, bbox_size):
        self.w_dim = [bbox_size[0], bbox_size[1], bbox_size[2]]


    def update_with_label(self, label = None):
        if label != None:
            self.label = label
        try:
            asking_for = '/obj/'+self.label
            aux_obj = rospy.get_param(asking_for)
            print 'AO: ', aux_obj
            self.weight = aux_obj['weight']
            self.dim = aux_obj['dimensions'] #[aux_obj['d'+str(i)] for i in range(1,4)]
            if 'placeability' in aux_obj.keys():
                self.placeability = aux_obj['placeability']
        #self.dim = [self.dim[_] + 0.02 for _ in range(3)] #A bit of margin
            self.dim.sort(reverse=True)
            return True
        except:
            print 'Not able to get parameters for label',self.label
            self.dim = [0.15,0.15,0.15]
            return False
        #TODO: update other parameters

    def update_pose_with_pos(self):
        if self.container == -1:
            print 'Cant update pose with pos bc container -1'
        self.pose[0:3] = [self.pos[0]/100., self.pos[1]/100., -rospy.get_param("/tote/height")]
        center_bin = get_params_yaml('bin'+self.container+'_pose')[0:3]
        self.pose[0:3] = [self.pose[i] + center_bin[i] for i in range(3)]
        self.pose[3:7] = [0, 0, 0, 1]
    ##########################
    ### Checking functions ###
    ##########################
    def get_box(self):
        #Assumes tote = 0, and bins = {1,2,3}
        return self.container - 4

    def in_tote(self):
        '''Check if the object is in the tote (Container zero'''
        return self.container == 0

    def in_shelf(self):
        '''Check if the object is in a shelf (Container nonzero)'''
        return self.container > 0
    #TODO: implement printing

