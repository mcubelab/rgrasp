#!/usr/bin/python

import gripper
import tf
from manual_fit.srv import *
import rospy
from grasping17 import grasp
import optparse
import time
import spatula
# Additional imports
from suction_down_simple import suction_down_simple
import realsense_camera.srv
from pr_msgs.srv import SuctionData1
import os
import suction

# Global variable to hold log file reference
log_file = 0

# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
# Instructions for collecting vision data: https://github.com/mcubelab/arc/wiki/Collecting-Vision-Data
# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
if __name__=='__main__':

    # ROS setup
    rospy.init_node('collectdata17', anonymous=True)

    # Parse inputs
    parser = optparse.OptionParser()
    parser.add_option('-p', '--passive', action='store_true', dest='for_passive_vision', help='Collecting data for passive vision or not', default=False)
    parser.add_option('-s', '--suction', action='store_true', dest='for_suction', help='Collecting data with suction or not', default=False)
    (opt, args) = parser.parse_args()
    for_passive_vision = opt.for_passive_vision
    for_suction = opt.for_suction
    # goToHome.goToARC(slowDown = True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.5)
    bin = '1';
    if for_passive_vision:
        camera_0 = rospy.get_param('/camera/bin' + bin + '_passive_far');
        camera_1 = rospy.get_param('/camera/bin' + bin + '_passive_near');
        log_file_path = os.environ['ARCDATA_BASE'] + '/collectdata/{}.passive.txt'.format(int(round(time.time())))
    else:
        camera_0 = rospy.get_param('/camera/bin' + bin + '_active_near');
        camera_1 = rospy.get_param('/camera/bin' + bin + '_active_far');
        log_file_path = os.environ['ARCDATA_BASE'] + '/collectdata/{}.active.txt'.format(int(round(time.time())))

    # Create log file for current capture data session
    log_file = open(log_file_path, 'w')


    captureRealSenseCameraSnapshot0 = rospy.ServiceProxy('/' + camera_0['machine'] + '/realsense_camera/capture', realsense_camera.srv.snapshot)
    captureRealSenseCameraSnapshot1 = rospy.ServiceProxy('/' + camera_1['machine'] + '/realsense_camera/capture', realsense_camera.srv.snapshot)

    # Capture background
    log_file.write('Calibration\n')
    if for_passive_vision:
        raw_input("Place calibration board in tote and press Enter...")
    else:
        raw_input("Place calibration stick in middle of tote and press Enter...")
    rgbd_frame_data = captureRealSenseCameraSnapshot0(camera_0['serial'])
    log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                     rgbd_frame_data.file_paths[2]))
    rgbd_frame_data = captureRealSenseCameraSnapshot1(camera_1['serial'])
    log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                     rgbd_frame_data.file_paths[2]))
    print('Captured images of calibration tool in tote!')

    raw_input("Remove calibration tool and press Enter...")
    log_file.write('Empty\n')
    rgbd_frame_data = captureRealSenseCameraSnapshot0(camera_0['serial'])
    log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                     rgbd_frame_data.file_paths[2]))
    rgbd_frame_data = captureRealSenseCameraSnapshot1(camera_1['serial'])
    log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                     rgbd_frame_data.file_paths[2]))
    print('Captured images of empty tote!')

    # Move robot to capturing data position
    if not for_passive_vision:
        rospy.wait_for_service('suction_service',timeout = 5)
        suction_service=rospy.ServiceProxy('suction_service', SuctionData1)
        if for_suction:
            suction_down_simple(listener=listener, br=br, flag=1,bin_id=1,print_messages=False)
        else:
            grasp(objInput = [], listener=listener, br=br, isExecute=True, binId= 1, flag = 1, withPause = False)

    # Capture data
    log_file.write('Data\n')
    while True:
        if not for_passive_vision:
            if for_suction:
                suction.start()
                raw_input("Put object under suction cup and press Enter to continue...")
            else:
                gripper.open()
                spatula.open()
                resp1=suction_service("g_open", "")
                raw_input("Put object in gripper and press Enter to continue...")
                # gripper.set_force(80)
                # gripper.grasp()
                # gripper.close()
                gripper.grasp_in(50,50)
                spatula.close()
                raw_input("Move your hand away and press Enter to continue...")
        else:
            raw_input("Rearrange objects in the tote and press Enter to continue...")

        rgbd_frame_data = captureRealSenseCameraSnapshot0(camera_0['serial'])
        log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                         rgbd_frame_data.file_paths[2]))
        rgbd_frame_data = captureRealSenseCameraSnapshot1(camera_1['serial'])
        log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                         rgbd_frame_data.file_paths[2]))
        print('Captured images!')

        if not for_passive_vision:
            raw_input("Get the object. WARNING: DO NOT PRESS ENTER BEFORE HOLDING OBJECT. IT WILL DAMAGE THE WEIGHT SENSOR. Press Enter to continue...")
            if for_suction:
                suction.stop()
                raw_input("Wait for suction tank to recover and press Enter to continue...")
            else:
                gripper.open()
                spatula.open()


    log_file.close()
