from collision_detection.collisionHelper import *

import rospy
import numpy as np
from numpy import linalg as la
from pr_msgs.msg import *
from manual_fit.srv import *
from copy import deepcopy
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
from collision_detection.collisionHelper import *
import tf

from ik.ik import *
from ik.helper import *
from ik.roshelper import *
from goToHome import *

#Collision Debug
rospy.init_node('collisionCheck', anonymous=True)
#~ collision, tcp_pos_grasp, hand_orient_norm_grasp = collisionCheck(.11, np.array([1,2,3]), np.array([[0,1,0],[0,0,1],[1,0,0]]))


listener = tf.TransformListener()
br = tf.TransformBroadcaster()

tmp = getToteSections(listener=listener, br=br);
#~ print 'sections',tmp

#~ array1, array2 = getToteBounds()
#~ print 'box1',array1
#~ print 'box2',array2

#~ ax = buildPlt()
#~ for lv1 in range(0,1):
	#~ updatePlt(tmp[lv1], ax, edgeCol = 'r', lineColor = 'b')
#~ destroyPlt()


