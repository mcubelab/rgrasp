#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from pr_msgs.srv import SuctionData1

ErrorMessage = 'Scorpion not connected, skipping command: '

exec_joint_pub = rospy.Publisher('/virtual_joint_states', JointState, queue_size=10)
haveraspberry = rospy.get_param('/use_raspberry', True)
fastvirtual = rospy.get_param('/fast_virtual', False)

#~If Raspberry is not connected, publish simulated value to rviz
def virtual_publish(command_name, position):
    print '[scorpion]', command_name, ErrorMessage, command_name
    # publish to joint state publisher for visualization without real hand
    jnames = ['scorpion_tail_bot_link_joint', 'scorpion_tail_top_link_joint','scorpion_tail_platform_joint']
    js = JointState()
    js.name  = jnames
    js.position = position
    exec_joint_pub.publish(js)
    if not fastvirtual:
        rospy.sleep(0.5)

def back():
    scorpion_back=rospy.ServiceProxy('/suction_service',SuctionData1)
    if haveraspberry:
        scorpion_back("sc_back","")
    else:
        virtual_publish(command_name='back', position = [-1.85, -1.81, 1.81])

def fastback():
    scorpion_back=rospy.ServiceProxy('/suction_service',SuctionData1)
    if haveraspberry:
        scorpion_back("sc_fastback","")
    else:
        virtual_publish(command_name='back', position = [-1.85, -1.81, 1.81])
           
def fwd():
    scorpion_fwd=rospy.ServiceProxy('/suction_service',SuctionData1)
    if haveraspberry:
        scorpion_fwd("sc_fwd","")
    else:
        virtual_publish(command_name='fwd', position= [0,0,0])
        
def fastfwd():
    scorpion_fwd=rospy.ServiceProxy('/suction_service',SuctionData1)
    if haveraspberry:
        scorpion_fwd("sc_fastfwd","")
    else:
        virtual_publish(command_name='fwd', position= [0,0,0])


    
if __name__=='__main__':
    rospy.init_node("scorpion_testing")
