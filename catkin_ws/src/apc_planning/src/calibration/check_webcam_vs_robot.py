#!/usr/bin/env python
# Get an array of 
# [3D apriltag pose from vicon in world frame, apriltag 3D position from detector in webcam frame]

# want to find the mapping between webcam frame wrt world.

# rosrun apc_planning check_webcam_vs_robot.py 612203002922
# rosrun apc_planning extract_corners_from_img.py
# rosrun apc_planning calib3d2d.py 612203002922

from ik.helper import *
from ik.roshelper import *
import tf.transformations as tfm
from rigid_transform_3D import rigid_transform_3D
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image
from robot_comm.srv import *
from realsense_camera.srv import snapshot
import pdb 

import os
import errno
def make_sure_path_exists(path):
    try:
        os.makedirs(path)
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise

pts_vicon = []
pts_apriltag = []

rospy.init_node('robot_vs_webcam')
listener = tf.TransformListener()
setCartRos = rospy.ServiceProxy('/robot1_SetCartesian', robot_SetCartesian)
getJoint = rospy.ServiceProxy('/robot1_GetJoints', robot_GetJoints)
setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
setZone = rospy.ServiceProxy('/robot1_SetZone', robot_SetZone)
setAcc = rospy.ServiceProxy('/robot1_SetAcc', robot_SetAcc)
setSpeed = rospy.ServiceProxy('/robot1_SetSpeed', robot_SetSpeed)
capture = {}
capture['arc_1'] = rospy.ServiceProxy('/arc_1/realsense_camera/capture', snapshot)
capture['arc_2'] = rospy.ServiceProxy('/arc_2/realsense_camera/capture', snapshot)

def setCart(pos, ori):
    param = (np.array(pos) * 1000).tolist() + ori
    #print 'setCart', param
    #pause()
    setCartRos(*param)
    
#setjoint -29.42 50.03 3.11 0 36.85 -164.42
# get joint and set joint 6 to zero
argv = sys.argv


cam_configs = {'612203002922': {'bin_num': 'bin0', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '614203000465': {'bin_num': 'bin0', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #'612203004574': {'bin_num': 'bin0', 'place': 'active_near', 'ns': 'arc_1'}, 
              #'616205005772': {'bin_num': 'bin0', 'place': 'active_far', 'ns': 'arc_1'},

              '616205001219': {'bin_num': 'bin1', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '61420501085': {'bin_num': 'bin1', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #'616205004776': {'bin_num': 'bin1', 'place': 'active_near', 'ns': 'arc_1'}, 
              #'614203003651': {'bin_num': 'bin1', 'place': 'active_far', 'ns': 'arc_1'},

              '612205002211': {'bin_num': 'bin2', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '612203004574': {'bin_num': 'bin2', 'place': 'passive_far', 'ns': 'arc_1'}, 
              #'614205001856': {'bin_num': 'bin2', 'place': 'active_near', 'ns': 'arc_1'}, 
              #'613201001839': {'bin_num': 'bin2', 'place': 'active_far', 'ns': 'arc_1'},

              '617205001931': {'bin_num': 'bin3', 'place': 'passive_near', 'ns': 'arc_1'}, 
              '612201002220': {'bin_num': 'bin3', 'place': 'passive_far', 'ns': 'arc_1'}, 
              '616203002024': {'bin_num': 'bin3', 'place': 'active_near', 'ns': 'arc_1'}, 
              '614204001012': {'bin_num': 'bin3', 'place': 'active_far', 'ns': 'arc_1'}}

tote_centers = {'bin0': [0.997, -0.393, 0.0632+0.06], 
               'bin1': [0.997, +0.019, 0.0632+0.06], 
               'bin2': [0.997, +0.425, 0.0632+0.06],  # need to change
               'bin3': [0.997, +0.569, 0.0632+0.06]}
limits_in_totes = {'passive_near': [-0.17, 0.19, -0.092, 0.092, 0.0547, 0.16],  # use calib stick
                   'passive_far': [-0.17, 0.15, -0.092, 0.092, 0.0547, 0.16],   # use calib stick
                   'active_near': [-0.19, 0.19, -0.092, 0.092, 0.17, 0.35],  # use calib stick
                   'active_far': [-0.19, 0.12, -0.092, 0.092, 0.28, 0.40]}  # use calib stick
                   
limits_in_totes = {'passive_near': [-0.26+ 0.06, 0.11, -0.139, 0.102, 0.1808, 0.2488], ## use hand # [xmin, xmax, ymin, ymax, zmin, zmax]
                   'passive_far': [-0.244+ 0.06, 0.128, -0.139, 0.073, 0.1818, 0.2498], ## use hand
                   'active_near': [-0.17, 0.17, -0.092, 0.092, 0.2, 0.38],  ## use hand
                   'active_far': [-0.17, 0.12, -0.092, 0.092, 0.31, 0.43]}  ## use hand
oris = {'passive_near': [0.0, 0.9239, -0.3827, 0],# use calib stick
        'passive_far': [0.0, 0.3827, 0.9239, 0], # use calib stick
        'active_near': [0.0, 0.9239, -0.3827, 0],# use calib stick
        'active_far': [0.0, 0.3827, 0.9239, 0]}# use calib stick
oris = {'passive_near': [0.0,0.7071,0.7071,0.0],  ## use hand
        'passive_far': [0.0,0.7071,-0.7071,0.0],   ## use hand
        'active_near': [0.0,0.7071,0.7071,0.0],  ## use hand
        'active_far': [0.0,0.7071,-0.7071,0.0]}  ## use hand

if len(argv) < 2:
    print 'check_webcam_vs_robot [camera serial number]'
    import pprint
    import operator
    pp = pprint.PrettyPrinter(indent=4)
    sorted_cam_configs = sorted(cam_configs.items(), key=lambda x: x[1]['bin_num'] + x[1]['place'])
    pp.pprint(sorted_cam_configs)
    exit(0)

if argv[1] in ['bin0', 'bin1', 'bin2', 'bin3']:
    cameraids = [camid for camid, cam_config in cam_configs.iteritems() if cam_config['bin_num'] == argv[1]]
else:
    cameraids = [argv[1]] #'612203002922'


for cameraid in cameraids:
    print 'calibrating', cameraid
    # parse the camera id to poses
    cam_config = cam_configs[ cameraid ]
    tote_center = tote_centers[ cam_config['bin_num'] ]
    limits_in_tote = limits_in_totes[ cam_config['place'] ]
            

    limits = [limits_in_tote[0]+tote_center[0], limits_in_tote[1]+tote_center[0], 
              limits_in_tote[2]+tote_center[1], limits_in_tote[3]+tote_center[1],
              limits_in_tote[4]+tote_center[2], limits_in_tote[5]+tote_center[2]]  #[xmin, xmax, ymin, ymax, zmin, zmax]
              
    
    
    nseg = [3, 3, 3]
    nrotate = 1
    #ori = [0, 0, 1, 0]
    ori = oris[ cam_config['place'] ]
    globalacc = 2             # some big number means no limit, in m/s^2
    zup = 0.6

    # prepare robot
    setZone(0)
    setSpeed(100, 60)
    setAcc(acc=globalacc, deacc=globalacc)
    j = getJoint()
    setJoint(j.j1, j.j2, j.j3, j.j4, j.j5, 0)
    setCart(tote_center[0:2] + [zup], ori)
    

    #~ if cam_config['bin_num'] == 'bin2':
        #~ if cam_config['place'] == 'passive_near':  
            #~ limits[0] = 0.815
            #~ limits[1] = 1.019
        #~ elif cam_config['place'] == 'passive_far':  
            #~ limits[0] = 0.953
            #~ limits[1] = 1.172
    
        
    #~ # use hand to calibrate  # also it is higher
    #~ if cam_config['bin_num'] == 'bin3':  
        #~ if cam_config['place'] == 'passive_near':
            #~ limits = [0.815, 1.019, 0.537, 0.693, 0.222, 0.331]
            #~ ori = [0,0.71,0.71,0]
        #~ if cam_config['place'] == 'passive_far':
            #~ limits = [0.953, 1.172, 0.487, 0.725, 0.262, 0.348]
            #~ ori = [0,0.71,-0.71,0]

    # prepare for image saving
    bridge = CvBridge()

    data = []
    save_dir = os.environ["DATA_BASE"] + "/camera_calib/" + cameraid + '/'

    import shutil
    try:
        shutil.rmtree(save_dir)
    except:
        pass

    make_sure_path_exists(save_dir)

    # turn to the right one
    capture[ cam_config['ns'] ](cameraid)

    timestr = 0
    for x in np.linspace(limits[1],limits[0], nseg[0]):
        for y in np.linspace(limits[2],limits[3], nseg[1]):
            for z in np.linspace(limits[5],limits[4], nseg[2]):
                setCart([x,y,z], ori)
                # get robot 3d point
                
                rospy.sleep(1)
                cross_poselist = lookupTransformList('/map','/cross_tip', listener)
                # take picture of camera
                
                point_cloud_xyz = capture[ cam_config['ns'] ](cameraid).point_cloud_xyz
                
                #~ msg = rospy.wait_for_message(cam_config['ns']+"/realsense", Image)
                msg = rospy.wait_for_message("/arc_1/rgb_bin0", Image)
                timestr = timestr + 1
                image_name = str(save_dir)+str(timestr)+".pgm"
                pointcloud_name = str(save_dir)+str(timestr)+".npy"
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(image_name, cv_image)
                
                np.save(pointcloud_name, point_cloud_xyz)
                
                data.append({"cross3d": cross_poselist, "pic_path": str(timestr)+".pgm", "pointcloud_path": str(timestr)+".npy"})
    
    setCart(tote_center[0:2] + [zup], ori)
    import json
    with open(save_dir+'data.json', 'w') as outfile:
        json.dump(data, outfile)


# point3d = [for p["cross3d"] in data]
# point2d = [for p["cross2d"] in data]
