#!/usr/bin/env python
#~ import ..gripper
import rospy
from ik.ik import GraspingGuard
# from ik.ik import SuctionGuard
#from ik.ik import WeightGuard

rospy.init_node('test_guard')


grasp_guard = GraspingGuard()
# suction_guard = SuctionGuard()
#weight_guard = WeightGuard(2)

grasp_guard.prep()
# suction_guard.prep()
#weight_guard.prep()


while not rospy.is_shutdown():
    rospy.sleep(0.1)
    print grasp_guard.test()
    # print suction_guard.test()
    #print weight_guard.test()

