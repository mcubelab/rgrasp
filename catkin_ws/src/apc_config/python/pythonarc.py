import sys
import os
sys.path.append(os.environ['RGRASP_BASE']+"/catkin_ws/src/apc_planning/src/")

import rospy
import tf
import numpy as np
import scorpion, suction, gripper, spatula
rospy.init_node('pythonrgrasp', anonymous=True)

listener = tf.TransformListener()
br = tf.TransformBroadcaster()
rospy.sleep(0.5)

print "Some cool code blocks:"
print " pubFrame(br, pose, 'obj', 'map')"
print " objPose = poseTransform(objPose, 'map', 'suction_shelf', listener)"
print " p = IKGuarded(target_tip_pos=[], target_tip_ori=[], tip_hand_transform=[]).plan()"
print " scorpion.fwd()"
print " scorpion.back()"
print " suction.start()"
print " suction.stop()"
print " gripper.open()"
print " gripper.close()"
print " spatula.open()"
print " spatula.close()"



#p = IKGuarded(target_tip_pos=[1.5552962738267355, -0.055624458593964907, 0.88940466239899996], target_tip_ori=[0.5,-0.5,0.5,-0.5], q0=[-0.06158292618, 0.05071129964, 0.5884785015, -0.09525217145, -0.6328909555, -1.495137224], tip_hand_transform=[0.0, 0.0, 0.339, 0, 0, 0]).plan()
#pause()
#p.execute()



