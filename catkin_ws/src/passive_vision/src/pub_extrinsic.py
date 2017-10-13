#!/usr/bin/env python
import tf.transformations as tfm
import tf
import numpy as np
from ik.roshelper import pubFrame
import rospy
from sensor_msgs.msg import CameraInfo
import sys
import os

def get_numbers_from_file(filename):
    ret = []
    with open(filename) as input_file:
        for line in input_file:
            line = line.strip()
            for number in line.split():
                ret.append( float(number) )
    return ret

def get_ext(cameraid):
    ret = get_numbers_from_file(os.environ['CODE_BASE'] + '/catkin_ws/src/passive_vision/camerainfo/' + cameraid + '.pose.txt')
    print cameraid, ret
    trans = [ret[3], ret[7], ret[11]]
    ret[3] = 0
    ret[7] = 0
    ret[11] = 0
    quat = tfm.quaternion_from_matrix(np.array(ret).reshape((4,4)))
    return trans + quat.tolist()

# 0. get camera id
cam_configs = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'}, 
              '612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'}, 
              '616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'}, 
              '616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'}, 
              '614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '616205002864': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_2'}, 
              '614205001856': {'bin_num': 'bin2', 'place': 'active_near', 'ns': 'arc_2'}, 
              '613201001839': {'bin_num': 'bin2', 'place': 'active_far', 'ns': 'arc_2'},

              '617205001931': {'bin_num': 'bin3', 'place': 'passive_near', 'ns': 'arc_2'}, 
              '612201002220': {'bin_num': 'bin3', 'place': 'passive_far', 'ns': 'arc_2'}, 
              '616203002024': {'bin_num': 'bin3', 'place': 'active_near', 'ns': 'arc_2'}, 
              '614204001012': {'bin_num': 'bin3', 'place': 'active_far', 'ns': 'arc_2'}}
if len(sys.argv) < 2:
    print 'pub_extrinsic [camera serial number]\n'
    import pprint
    pp = pprint.PrettyPrinter(indent=4)
    pp.pprint(cam_configs)
    exit(0)

cameraid = sys.argv[1]

# 1. get extrinsics from file
ext = get_ext(cameraid)

# 2. publish the frame of realsense_frame

rospy.init_node('pub_extrinsic')
br = tf.TransformBroadcaster()
rospy.sleep(1)
while not rospy.is_shutdown():
    print ext
    # publish the specified
    pubFrame(br, ext, 'realsense', 'map', 1)
    
    # publish all
    for cam_id, cam in cam_configs.iteritems():
        ext_ = get_ext(cam_id)
        pubFrame(br, ext_, 'realsense_' + cam_id, 'map', 1)
    
    rospy.sleep(1)


#rosrun tf static_transform_publisher 0.48641 -0.55125 0.26128 0.6471507745494862 -0.6314757179800774 0.2911898701368086, -0.3095337406332384 map realsense 100

#~ c = CameraInfo()
#~ c.header.frame_id = 'realsense'
#~ c.height = 480
#~ c.width = 640
#~ c.K = [6.16917053e+02,  0.00000000e+00,  3.16593872e+02,
 #~ 0.00000000e+00,  6.16917175e+02,  2.36076706e+02,
 #~ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]

#~ c.P = [6.16917053e+02,  0.00000000e+00,  3.16593872e+02, 0,
 #~ 0.00000000e+00,  6.16917175e+02,  2.36076706e+02, 0,
 #~ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00, 0]

#~ pub = rospy.Publisher('/realsense/camera_info', CameraInfo, queue_size=10)

#~ while True:
    #~ pub.publish(c)
    #~ rospy.sleep(1)
