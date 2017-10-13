#!/usr/bin/env python
import rospy
from realsense_camera.srv import snapshot
from ik.helper import Timer
rospy.init_node('robot_vs_webcam')
capture = rospy.ServiceProxy('/arc_1/realsense_camera/capture', snapshot)
import numpy as np
import struct
import cv2

cams = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #'612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'}, 
              #'616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'}, 
              '616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'}, 
              '614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612203004574': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_2'}, 
              '614205001856': {'bin_num': 'bin2', 'place': 'active_near', 'ns': 'arc_2'}, 
              '613201001839': {'bin_num': 'bin2', 'place': 'active_far', 'ns': 'arc_2'},

              '613204000977': {'bin_num': 'bin3', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612201002220': {'bin_num': 'bin3', 'place': 'passive_far', 'ns': 'arc_2'}, 
              '616203002024': {'bin_num': 'bin3', 'place': 'active_near', 'ns': 'arc_2'}, 
              '614204001012': {'bin_num': 'bin3', 'place': 'active_far', 'ns': 'arc_2'}}

import sys
camids = ['616205001219']
times = 1

camids = [camid for camid,_ in cams.iteritems()]
if len(sys.argv) == 2:
    camids = [sys.argv[1]]
    times = 100
        
for i in xrange(times):
    for camid in camids:
        rospy.loginfo('capturing %s  time %d' , camid, i+1)
        cv2.namedWindow("imgrgb");
        cv2.namedWindow("imgdepth");
        cv2.moveWindow("imgdepth", 0,500);
        with Timer():
            res = capture(camid)
        
        try:
            rows = 480
            cols = 640
            value = struct.unpack('B'*rows*cols*3, res.point_cloud_rgb)
            img = np.array(value, dtype=np.uint8)
            #img = img / 255.0
            img = img.reshape((rows, cols,3))
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            cv2.imshow('imgrgb',img)
            
            depthimg = np.array(res.point_cloud_xyz)
            depthimg = depthimg.reshape((rows, cols,3))
            cv2.imshow('imgdepth',depthimg[:,:,2])
            
            key = cv2.waitKey(200)
        
            #print max(res.point_cloud_xyz)
            #print res.point_cloud_rgb[0]
            #print res.color_camera_intrinsics[0:9]
            rospy.loginfo('success capturing '+camid)
        except:
            rospy.logwarn('error capturing '+ camid)
        print ''
        

