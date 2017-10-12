#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from pr_msgs.srv import SuctionData1

ErrorMessage = 'Spatula not connected, skipping command: '
exec_joint_pub = rospy.Publisher('/virtual_joint_states', JointState, queue_size=10)
gripper_command_pub = rospy.Publisher('/hand_commands', JointState, queue_size=10)

havegripper = rospy.get_param('/have_gripper', True)
haveraspberry = rospy.get_param('/use_raspberry', True)
fastvirtual = rospy.get_param('/fast_virtual', False)

def open():
    command = 'open'
    spatula_open=rospy.ServiceProxy('/suction_service',SuctionData1)
    
    if haveraspberry:
        spatula_open("g_open","")
        print '[Spatula] open'
        #~frank hack to publishn and record sent spatula commands data
        jnames = ['spatula_joint']
        js = JointState()
        js.name  = jnames
        js.position = [0]
        gripper_command_pub.publish(js)
    else:
        #~If spatula is not connectect, publish simulated value to rviz
        print '[Spatula] open,', ErrorMessage, command
        # publish to joint state publisher for visualization without real hand
        jnames = ['bot_spatula_joint']
        js = JointState()
        js.name  = jnames
        js.position = [-0.4]
        exec_joint_pub.publish(js)
        if not fastvirtual:
            rospy.sleep(0.5)
       
def close():  
    command = 'close'
    spatula_close=rospy.ServiceProxy('/suction_service',SuctionData1)
    if haveraspberry:
        spatula_close("g_close","")
        print '[Spatula] close'
        #~frank hack to publishn and record sent spatula commands data
        jnames = ['spatula_joint']
        js = JointState()
        js.name  = jnames
        js.position = [-0.4]
        gripper_command_pub.publish(js)
    else:
        print '[Spatula] close,', ErrorMessage, command
        # publish to joint state publisher for visualization without real hand
        jnames = ['bot_spatula_joint']
        js = JointState()
        js.name  = jnames
        js.position = [0.1]
        exec_joint_pub.publish(js)
        if not fastvirtual:
            rospy.sleep(0.5)
    
if __name__=='__main__':
  rospy.init_node("spatula_testing")
