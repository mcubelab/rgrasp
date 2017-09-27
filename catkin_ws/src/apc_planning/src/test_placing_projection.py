import rospy
from suction_projection import suction_projection_func
import tf.transformations as tfm
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import tf
import time
from ik.helper import poseTransform
from ik.helper import vision_transform_precise_placing
from ik.helper import pose_transform_precise_placing
from collision_detection.collisionHelper import getBinPoints
from collision_detection.collisionHelper import getFingerPoints
from ik.helper import vision_transform_precise_placing_with_visualization
from visualization_msgs.msg import MarkerArray
import suction_down_simple

if __name__=="__main__":
    rospy.init_node('test_stuff')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.3)

    box_rot=tfm.euler_matrix(0, 0, 40.0*np.pi/(180.0), axes='sxyz')
    box_rot[:,2]=-box_rot[:,2]
    box_rot[:,1]=-box_rot[:,1]
    box_quat=tfm.quaternion_from_matrix(box_rot).tolist()
    box_pos=[.06,0.0,.5]
    box_pose=box_pos+box_quat

    place_rot=tfm.euler_matrix(0, 0, 30.0*np.pi/(180.0), axes='sxyz')
    place_quat=tfm.quaternion_from_matrix(place_rot).tolist()
    place_pos=[1.,0,0.0]
    place_pose=place_pos+place_quat

    #print box_pose
    map_box_pose = poseTransform(box_pose, "link_6", "map", listener)
    box_dim=[.07,.03,.01]
    bbox_info=map_box_pose[3:7]+map_box_pose[0:3]+box_dim

    (rel_pose,BoxBody)=vision_transform_precise_placing (bbox_info=bbox_info, listener=listener)
    viz_pub=rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    rospy.sleep(0.5)
    vision_transform_precise_placing_with_visualization(bbox_info=bbox_info,viz_pub=viz_pub,listener=listener)
    rospy.sleep(0.5)

    #making sure that rel_pose is correct
    #print np.linalg.norm(np.array(box_pose)-np.array(rel_pose))
    
    BoxBody_array=np.vstack(BoxBody)
    Box_hull = ConvexHull(BoxBody_array[:,0:2])

    #plt.plot(0.0, 0.0, 'go')
    #plt.plot(rel_pose[0], rel_pose[1], 'ro')
    #for simplex in Box_hull.simplices:
    #    plt.plot(BoxBody_array[simplex, 0], BoxBody_array[simplex, 1], 'k-')
    #plt.axis('equal')
    #plt.show()

    #suction cups of old hand facing right, fingers pointing down
    base_pose = [0.0,0.0,0.0]+[0, 1, 0,0]
    bin_pts = getBinPoints(binId=3, listener=listener, br=br)
    suc_down_quat= tfm.quaternion_matrix(base_pose[3:7])
    hand_orient_norm = suc_down_quat[0:3,0:3]
    hand_opening=0.0
    finger_pts= getFingerPoints(hand_opening, [0.0,0.0,0.0], hand_orient_norm, True)

    box_pos_final=tfm.translation_matrix(place_pose[0:3])
    box_pos_final=box_pos_final[0:3,3]
    box_rot_final=tfm.quaternion_matrix(place_pose[3:7])
    box_x_final=box_rot_final[0:3,0]
    box_y_final=box_rot_final[0:3,1]
    box_z_final=box_rot_final[0:3,2]
    b = [[1.0,1.0,1.0],[1.0,1.0,-1.0],[1.0,-1.0,1.0],[1.0,-1.0,-1.0],[-1.0,1.0,1.0],[-1.0,1.0,-1.0],[-1.0,-1.0,1.0],[-1.0,-1.0,-1.0]]
    BoxBody_final=[]
    for i in range(0, 8):   
        BoxBody_final.append(box_pos_final+box_dim[0]*box_x_final*b[i][0]/2.0+box_dim[1]*box_y_final*b[i][1]/2.0+box_dim[2]*box_z_final*b[i][2]/2.0)
    BoxBody_final=np.vstack(BoxBody_final)
    Box_hull_final = ConvexHull(BoxBody_final[:,0:2])

    #plt.plot(rel_pose[0], rel_pose[1], 'ro')
    for simplex in Box_hull_final.simplices:
        plt.plot(BoxBody_final[simplex, 0], BoxBody_final[simplex, 1], 'r-')

    drop_pose=pose_transform_precise_placing(rel_pose,BoxBody,place_pose,base_pose,bin_pts,finger_pts,margin=.03,show_plot=True)

    print drop_pose

    box_link6=tfm.quaternion_matrix(rel_pose[3:7])
    #box_link6=box_link6[0:3,0:3]
    
    
    
    suc_down_quat_final= tfm.quaternion_matrix(drop_pose[3:7])
    hand_orient_norm_final = suc_down_quat_final[0:3,0:3]
    finger_pts_final= getFingerPoints(hand_opening, drop_pose[0:3], hand_orient_norm_final, True)

    finger_hull_final = ConvexHull(finger_pts_final[:,0:2])
    for simplex in finger_hull_final.simplices:
        plt.plot(finger_pts_final[simplex, 0], finger_pts_final[simplex, 1], 'g-')

    pose_transform_precise_placing(rel_pose,BoxBody,place_pose,base_pose,bin_pts,finger_pts,margin=.03,show_plot=True)
    
    print 'hello'

    print np.array(place_pose[3:7])-np.array(tfm.quaternion_from_matrix(np.dot(suc_down_quat_final,box_link6)))
    #print bin_pts
    #print finger_pts
    
    #suction_down_simple.suction_down_simple(listener=listener, br=br, withPause=True,
    #suction_position_target=[-3,-5,-5],
    #surface_normal=[0,0,1.0],
    #flag=0,
    #bin_id=0,
    #print_messages=False,
    #obj_ID='expo_eraser',
    #rel_pose=rel_pose,
    #BoxBody=BoxBody,
    #place_pose=place_pose,
    #box_height=0.1,
    #listener=None)
    
