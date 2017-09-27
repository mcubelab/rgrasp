#!/usr/bin/env python

import sys
import optparse
import numpy as np
import rospkg
import rospy
import tf
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
import numpy as np
from ik.roshelper import *
from ik.helper import *
import tf.transformations as tfm
from sensor_msgs.msg import LaserScan
from apc_sensor.srv import *
import math
import yaml
from ik.ik import setCart, setSpeed, setAcc, setJoint
from manual_fit.srv import SetShelfPose
from ctypes import cdll, c_void_p, c_int

def object_calib(object_id = 'shelf_icp', nmeasure = 10):
    lib = cdll.LoadLibrary(os.environ['APC_BASE'] + '/catkin_ws/src/apc_planning/src/c++/libpcprocess.so')
    bestPoint = lib['bestPoint']
    bestPoint.argtypes = [c_void_p, c_int, c_void_p, c_void_p]
    listener = tf.TransformListener()
    getPointCloud = rospy.ServiceProxy('/realsense/apc_sensor', StreamSensor)
    setPose_ = rospy.ServiceProxy('/set_shelf_pose', SetShelfPose)
    rospy.sleep(0.1)
    
    if object_id == 'shelf_icp':
        tags_object_poses = [{'id':0, 'pose':[]}, {'id':1, 'pose':[]}] 
        pointcloud_topic = '/realsense/pointcloud' 
        
    #setAcc(28,8)
    setAcc(5,5)
    setSpeed(2400,600)
    
    setJoint = rospy.ServiceProxy('/robot1_SetJoints', robot_SetJoints)
    getJoint = rospy.ServiceProxy('/robot1_GetJoints', robot_GetJoints)
            
    # Algorithm as follows
    # for each apriltag on object:
      # 0. move to a place can see the tag
      # for each image:
        # 1. find apriltag in map as A
        # 2. get object frame in apriltag frame as B
        # 3. concatenate 2: A*B
        # 4. append the trasform into array: 
    # 5. set the object frame as the average
    # 6. put it in a yaml file
    # 7. restart some process to reload the parameter.

    
    def findtagid(detections, tag_obj_id):
        for d in detections:
            if d.id == tag_obj_id:
                return d.pose
        return None
    
    obj_map_poses = []
    bestRightPoints = []
    bestLeftPoints = []
    
    origJoint = getJoint()
    #pause()
    measured_shelfz = -0.53257 # in lab
    # real competition :-0.517024695873
    half_shelf_thick = 0.435
            
    #rospy.sleep(1)
    for tag_obj in tags_object_poses:
        tag_obj_pose = tag_obj['pose']
        tag_obj_id = tag_obj['id']
        # move the robot if necessary 
        
        if tag_obj_id == 0:
            setJoint(-28.56,11.27,17.86,-48.19,-39.9,130.62)
            #setCart([0.99,-0.43,0.95753],[0.5,0.5,0.5,0.5])
        elif tag_obj_id == 1:
            #setJoint(28.56,11.27,17.86,48.19,-39.9,49.38)
            setCart([0.99,0.43,0.95753],[0.5,0.5,0.5,0.5])
        
        rospy.sleep(0.8)
        
        tf_cam_map = lookupTransformList('/map' , '/realsense_rgb_optical_frame', listener)
        T_mat = tfm.concatenate_matrices( tfm.translation_matrix(tf_cam_map[0:3]), tfm.quaternion_matrix(tf_cam_map[3:7]))
        T_mat = T_mat[0:3][:,0:4]
        for i in xrange(nmeasure): 
            with Timer('get pc'):
                print '[Calib] wait for pointcloud at', pointcloud_topic
                pointcloud = getPointCloud()  # hack
                pc = pointcloud.cloudXYZ

            pc = np.array(pc)                
            bestRightPoint = np.array((0.0,0.0,0.0))
            bestLeftPoint = np.array((0.0,0.0,0.0))
            
            # import json
            # with open('data%d_%d.txt' % (tag_obj_id,i), 'w') as outfile:
                # json.dump(pointcloud.cloudXYZ, outfile)
            # with open('data%d_%d_T.txt' % (tag_obj_id,i), 'w') as outfile:
                # json.dump(T_mat.tolist(), outfile)
            #rospy.sleep(0.3)
                
            with Timer('process pc'):
                if tag_obj_id == 0:
                    bestPoint(pc.ctypes.data, tag_obj_id, T_mat.flatten().ctypes.data, bestRightPoint.ctypes.data)
                elif tag_obj_id == 1:
                    bestPoint(pc.ctypes.data, tag_obj_id, T_mat.flatten().ctypes.data, bestLeftPoint.ctypes.data)
                # if tag_obj_id == 0:
                    # 
                    # center = (1.467, -0.42714, 0.92472)
                    # for j in xrange(0,480*640):
                        # pos_cam = pointcloud.cloudXYZ[j*3:j*3+3]
                        # if abs(x)<1e-9 and abs(y)<1e-9 and abs(z)<1e-9:
                            # continue
                        # (x,y,z) = np.dot(T_mat, np.array(pos_cam + (1,))).tolist()
                        # if abs(x-center[0]) < 0.08 and abs(y-center[1]) < 0.06 and abs(z-center[2]) < 0.05 and y < bestRightPoint[2] :
                            # bestRightPoint = (x,y,z)
                            # 
                # elif tag_obj_id == 1:
                    # center = (1.467, 0.42714, 0.92472)
                    # for j in xrange(0,480*640):
                        # pos_cam = pointcloud.cloudXYZ[j*3:j*3+3]
                        # if abs(x)<1e-9 and abs(y)<1e-9 and abs(z)<1e-9:
                            # continue
                        # (x,y,z) = np.dot(T_mat, np.array(pos_cam + (1,))).tolist()
                        # 
                        # if abs(x-center[0]) < 0.08 and abs(y-center[1]) < 0.06 and abs(z-center[2]) < 0.05 and y > bestLeftPoint[2] :
                            # bestLeftPoint = (x,y,z)
                            
            if tag_obj_id == 0 and abs(bestRightPoint[0]) > 1e-9:
                bestRightPoints.append(bestRightPoint.tolist())
            elif tag_obj_id == 1 and abs(bestLeftPoint[0]) > 1e-9:
                bestLeftPoints.append(bestLeftPoint.tolist())
            
    setJoint(origJoint.j1,origJoint.j2,origJoint.j3,origJoint.j4,origJoint.j5,origJoint.j6)
    
    # 5. set the object frame from avg(bestRightPoints), avg(bestLeftPoints)
    # do things in 2d, ignore z
    print 'bestRightPoints', bestRightPoints
    print 'bestLeftPoints', bestLeftPoints
    bestRightPoint = np.average(np.array(bestRightPoints), axis=0).tolist()
    bestLeftPoint = np.average(np.array(bestLeftPoints), axis=0).tolist()
    
    lp, rp = np.array(bestLeftPoint[0:2]), np.array(bestRightPoint[0:2])
    mp = (lp+rp)/2 # middle point in front face
    lr = (lp-rp)   # a vector
    
    normal = np.array([lr[1], -lr[0]])
    normal = normal / np.linalg.norm(normal) # pointing from robot to shelf
    
    centerp = mp + normal * half_shelf_thick
    
    avg_obj_map_pose = centerp.tolist() + [measured_shelfz] + tfm.quaternion_about_axis(np.pi/2 + np.arctan2(normal[1], normal[0]), [0,0,1]).tolist()
    print 'avg=', avg_obj_map_pose

    
    rospack = rospkg.RosPack()
    def set_the_result(name):
        prefix = '/' + name
        d = {prefix+'/x':avg_obj_map_pose[0], prefix+'/y':avg_obj_map_pose[1], 
             prefix+'/z':avg_obj_map_pose[2],
             prefix+'/qx':avg_obj_map_pose[3], prefix+'/qy':avg_obj_map_pose[4], 
             prefix+'/qz':avg_obj_map_pose[5], prefix+'/qw':avg_obj_map_pose[6]}
        
        # 6.1 put it in a yaml file
        with open(rospack.get_path('apc_config')+prefix+'.yaml', 'w') as yaml_file:
            yaml_file.write( yaml.dump(d, default_flow_style=False))
            
        # 6.2 update rosparam
        rospy.set_param(prefix+'/x', avg_obj_map_pose[0])
        rospy.set_param(prefix+'/y', avg_obj_map_pose[1])
        rospy.set_param(prefix+'/z', avg_obj_map_pose[2])
        rospy.set_param(prefix+'/qx', avg_obj_map_pose[3])
        rospy.set_param(prefix+'/qy', avg_obj_map_pose[4])
        rospy.set_param(prefix+'/qz', avg_obj_map_pose[5])
        rospy.set_param(prefix+'/qw', avg_obj_map_pose[6])
    
    #print 'Do you want to save and update the calibration result?'
    #pause()
    print 'Save the calibration to file'
    
    # 6. save result to file and rosparam
    set_the_result('shelf_pose')
    
    # 7. restart some process to reload the parameter.
    setPose_(poselist2pose(avg_obj_map_pose))
        
def main(argv=None):
    rospy.init_node('object_calib', anonymous=True)
    

    parser = optparse.OptionParser()
    parser.add_option('', '--nmeasure', action="store", dest='nmeasure', type='int',
                      help='How many measure to take for each tag', 
                      default=10)  

    (opt, args) = parser.parse_args()
    if len(args) < 1:
        print 'Usage: object_calib.py shelf'
        
    object_calib(args[0], opt.nmeasure)
    
    

if __name__=='__main__':
    main(sys.argv)
