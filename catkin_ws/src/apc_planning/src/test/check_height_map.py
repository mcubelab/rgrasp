#!/usr/bin/python
import rospy
from ik.helper import get_params_yaml
import sys
from colorama import Fore, Back, Style
import colorama
from explainer import Explainer
import numpy as np
import math
from math import ceil, floor
import optparse
import copy
from item import Item
import matplotlib.image as img
import matplotlib.pyplot as plt
from PIL import Image
import os
import time
import json
try:
    import passive_vision.srv
    import active_vision.srv
    import realsense_camera.srv
except:
    print 'FAILED TO IMPORT VISION, WILL ONLY RUN IN VIRTUAL'

# import profile
# profile.run('main()')

def main():
    getPassiveVisionEstimate = rospy.ServiceProxy('/passive_vision/estimate', passive_vision.srv.state)
    test_bin = 1
    num_bins = 3
    disc = 0.01
    vis_disc = 0.002
    tote_length = rospy.get_param("/tote/length")/0.61*0.6 # a bit of safety
    tote_width = rospy.get_param("/tote/width")
    tote_height = rospy.get_param("/tote/height")
    h = [tote_height for _ in range(3)]
    h[0]/=2
    h[1]/=2
    d = [[tote_length, tote_width] for _ in range(3)]
    discDim = [[int(math.floor(x[0]/disc)), int(math.floor(x[1]/disc))] for x in d]#discretized dimensions

    for i in range(0, 10):
        passive_vision_state = getPassiveVisionEstimate('',test_bin)

    M = passive_vision_state.height_map
    outfilename = os.environ['ARCDATA_BASE']+'/height_map/heigh_map%d.json'%int(time.time())
    DATA = {}
    DATA['height_map'] = M
    M = np.asarray(M)
    with open(outfilename, 'w') as outfile:
        json.dump(DATA, outfile, sort_keys = True, indent = 4, ensure_ascii=False)

    M = M.reshape((200,300))
    M = M.transpose()
    print discDim[0][0], discDim[0][1]
    HeightMap = [np.zeros([discDim[i][0], discDim[i][1]]) for i in range(num_bins)] #Height Maps
    HM = np.multiply(HeightMap[test_bin], 100) #Multiply each element by 100

    for i in range(len(HM)-1):
        for j in range(len(HM[i])-1):
            [x_ini, x_fin, y_ini, y_fin] = [int(floor(i*disc/vis_disc)), int(floor((i+1)*disc/vis_disc)), int(ceil(j*disc/vis_disc)), int(ceil((j+1)*disc/vis_disc))]
            x_fin = max(x_ini+1, x_fin)
            y_fin = max(y_ini+1, y_fin)
            #print i,',',j,': ',len(HM),',',len(HM[0]),':',x_ini, x_fin, y_ini, y_fin, len(M), len(M[0])
            HM[i][j] = np.max(M[[x_ini,x_fin],:][:,[y_ini,y_fin]])


    import matplotlib.pyplot as plt

    binListing = range(num_bins)
    plt.figure(1)
    plt.clf()
    plt.cla()
    plt.close()

    for i in range(1): #binListing:
        (y_dim, x_dim) = HM.shape

        subplot_number = int('1{}{}'.format(len(binListing), i+1))
        plt.subplot(subplot_number)
        heatmap = plt.pcolor(HM)

        plt.axis([0, x_dim, y_dim, 0])
    plt.savefig('heightmap_%d.png'%int(time.time()), dpi=100)
    plt.show()

if __name__ == "__main__":
    main()


