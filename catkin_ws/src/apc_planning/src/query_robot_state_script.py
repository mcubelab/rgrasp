import rospy
import sensor_msgs.msg
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import getCart


#returns (cartesian pose, joint angles)    
def query_robot_state_script():
    joint_topic = '/joint_states'
    rate = rospy.Rate(10) # 10hz
    rospy.sleep(0.5)
    
    cart_pos = getCart()
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg()
        q0 = APCrobotjoints.position
        
        if len(q0) >= 6:
            q0 = q0[0:6]   # take first 6, because in virtual environmet there will be additional 2 hand joint
            break
    print 'cart pos:'
    print cart_pos
    print 'joint angles:'
    print q0
    return (cart_pos,q0)

if __name__=="__main__":
    rospy.init_node('query_robot_state')
    query_robot_state_script()
