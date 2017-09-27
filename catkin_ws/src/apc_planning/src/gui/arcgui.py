#!/usr/bin/python

import Tkinter
import sys
import os
sys.path.append(os.environ['ARC_BASE']+"/catkin_ws/src/apc_planning/src/")

import rospy
import tf
import numpy as np
import scorpion, suction, gripper, spatula
import goToHome
from subprocess import call

rospy.init_node('gui', anonymous=True)

top = Tkinter.Tk()

def goarc():
    goToHome.goToARC(slowDown = True)
    
def testcam():
    call('rosrun realsense_camera testcam.py')

B1 = Tkinter.Button(top, text = 'gripper open', command = gripper.open)
B1.pack()
B2 = Tkinter.Button(top, text = 'gripper close', command = gripper.close)
B2.pack()
B3 = Tkinter.Button(top, text = 'scorpion fwd', command = scorpion.fwd)
B3.pack()
B4 = Tkinter.Button(top, text = 'scorpion back', command = scorpion.back)
B4.pack()
B5 = Tkinter.Button(top, text = 'spatula open', command = spatula.open)
B5.pack()
B6 = Tkinter.Button(top, text = 'spatula close', command = spatula.close)
B6.pack()
B7 = Tkinter.Button(top, text = 'goarc', command = goarc)
B7.pack()
B8 = Tkinter.Button(top, text = 'testcam', command = testcam)
B8.pack()

top.mainloop()
#rospy.spin()
