#!/usr/bin/python

from manual_fit.srv import *
import rospy
import time
import realsense_camera.srv
import os


# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
# Instructions for collecting vision data: https://github.com/mcubelab/arc/wiki/Collecting-Vision-Data
# ----------------------------------------------------------------------
# ----------------------------------------------------------------------
if __name__=='__main__':

    # ROS setup
    rospy.init_node('collectdata17', anonymous=True)
    bin = '0'
    camera_0 = rospy.get_param('/camera/bin' + bin + '_passive_far');
    camera_1 = rospy.get_param('/camera/bin' + bin + '_passive_near');

    log_file_path = os.environ['ARCDATA_BASE'] + '/collectdata/{}.passive.txt'.format(int(round(time.time())))
    with open(log_file_path, 'w') as log_file:
        captureRealSenseCameraSnapshot0 = rospy.ServiceProxy('/' + camera_0['machine'] + '/realsense_camera/capture', realsense_camera.srv.snapshot)
        captureRealSenseCameraSnapshot1 = rospy.ServiceProxy('/' + camera_1['machine'] + '/realsense_camera/capture', realsense_camera.srv.snapshot)
        # Capture background
        log_file.write('Calibration\n')
        rgbd_frame_data = captureRealSenseCameraSnapshot0(camera_0['serial'])
        log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                         rgbd_frame_data.file_paths[2]))
        rgbd_frame_data = captureRealSenseCameraSnapshot1(camera_1['serial'])
        log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                         rgbd_frame_data.file_paths[2]))
        print('Captured images of calibration tool in tote!')
        log_file.write('Empty\n')
        rgbd_frame_data = captureRealSenseCameraSnapshot0(camera_0['serial'])
        log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                         rgbd_frame_data.file_paths[2]))
        rgbd_frame_data = captureRealSenseCameraSnapshot1(camera_1['serial'])
        log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                         rgbd_frame_data.file_paths[2]))
        print('Captured images of empty tote!')
        # Capture data
        log_file.write('Data\n')
        while True:
            raw_input("Rearrange objects in the tote and press Enter to continue...")

            rgbd_frame_data = captureRealSenseCameraSnapshot0(camera_0['serial'])
            log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                             rgbd_frame_data.file_paths[2]))
            rgbd_frame_data = captureRealSenseCameraSnapshot1(camera_1['serial'])
            log_file.write('{}\n{}\n'.format(rgbd_frame_data.file_paths[1],
                                             rgbd_frame_data.file_paths[2]))
            print('Captured images!')
