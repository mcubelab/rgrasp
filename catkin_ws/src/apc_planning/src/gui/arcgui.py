#!/usr/bin/python

import Tkinter
import sys
import os
sys.path.append(os.environ['CODE_BASE']+"/catkin_ws/src/apc_planning/src/")

import rospy
import tf
import numpy as np
import gripper, spatula
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
B3 = Tkinter.Button(top, text = 'spatula open', command = spatula.open)
B3.pack()
B4 = Tkinter.Button(top, text = 'spatula close', command = spatula.close)
B4.pack()
B5 = Tkinter.Button(top, text = 'goarc', command = goarc)
B5.pack()
B6 = Tkinter.Button(top, text = 'testcam', command = testcam)
B6.pack()

top.mainloop()
#rospy.spin()
