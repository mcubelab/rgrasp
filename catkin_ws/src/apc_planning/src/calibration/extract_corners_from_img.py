#!/usr/bin/env python


import json
import cv2
import os
import copy
from numpy import array as npa
import numpy as np
import sys

  
image = []
image_viz = []
refPt = (0,0)
color = (0,255,255)
color2 = (0,0,255)
def click(event, x, y, flags, param):
    global refPt
    global image
    global image_viz
    
    if event == cv2.EVENT_LBUTTONUP:
    # record the ending (x, y) coordinates and indicate that
    # the cropping operation is finished
        refPt = (x, y)
        image_viz = copy.deepcopy(image)
        cv2.line(image_viz, refPt, refPt, color)
        cv2.imshow("image", image_viz)
        print 'clicked point', refPt

def show_updated(pt):
    global image_viz
    print pt
    pt_int = tuple([int(round(p)) for p in pt])
    print pt_int
    cv2.line(image_viz, (pt_int[0]-2, pt_int[1]), (pt_int[0]+2, pt_int[1]), color2)
    cv2.line(image_viz, (pt_int[0], pt_int[1]-2), (pt_int[0], pt_int[1]+2), color2)
    cv2.imshow("image", image_viz)


cam_configs = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #'612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'}, 
              #'616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #~ '616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'}, 
              #~ '614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612203004574': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_2'}, 
              #~ '614205001856': {'bin_num': 'bin2', 'place': 'active_near', 'ns': 'arc_2'}, 
              #~ '613201001839': {'bin_num': 'bin2', 'place': 'active_far', 'ns': 'arc_2'},

              '617205001931': {'bin_num': 'bin3', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612201002220': {'bin_num': 'bin3', 'place': 'passive_far', 'ns': 'arc_2'}}
              #~ '616203002024': {'bin_num': 'bin3', 'place': 'active_near', 'ns': 'arc_2'}, 
              #~ '614204001012': {'bin_num': 'bin3', 'place': 'active_far', 'ns': 'arc_2'}}


argv = sys.argv
if argv[1] in ['bin0', 'bin1', 'bin2', 'bin3']:
    cameraids = [camid for camid, cam_config in cam_configs.iteritems() if cam_config['bin_num'] == argv[1]]
else:
    cameraids = [argv[1]] #'612203002922'


for cameraid in cameraids:
    print 'calibrating', cameraid
    save_dir = os.environ["DATA_BASE"] + "/camera_calib/" + cameraid + '/'
    with open(save_dir+'data.json') as data_file:    
        data = json.load(data_file)

    newdata = []
    for d in data:
        try:
            image = cv2.imread(save_dir + d['pic_path'])
            cv2.namedWindow("image")
            cv2.setMouseCallback("image", click)
            cv2.imshow('image',image)
        except:
            continue
        #(x,y) = getClick()
        key = ''
        while True:
            # display the image and wait for a keypress
            key = cv2.waitKey(3) & 0xFF
            if key == ord("n"):
                break
            if key == ord("c"):  # skip it
                break
        
        if key == ord("c"):  # skip it (maybe pattern not in view)
            continue
        
        corners = np.float32([refPt])
        print 'clicked' , refPt
        d["cross2d"] = corners.tolist()[0]
        newdata.append(d)


    import json
    with open(str(save_dir)+'data.extracted2d.json', 'w') as outfile:
        json.dump(newdata, outfile)
