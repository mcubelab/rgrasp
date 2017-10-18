//
// Created by mcube on 6/28/17.
//

#ifndef PROJECT_STREAM_H
#define PROJECT_STREAM_H
#include "ros/ros.h"

#include "realsense_camera/snapshot.h"
#include "realsense_camera/start_lasers.h"
#include <librealsense/rs.hpp>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

#include <ctime>
#include <iostream>
#include <dirent.h>
#include <unistd.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include "helper.h"

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <ros/console.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

using pcl::PointXYZRGB;
typedef PointXYZRGB PointType;

// Constant values
const unsigned int microseconds = 100000;
const std::string data_directory = "/home/mcube/arcdata/";
const std::string camerainfo_directory = "/home/mcube/rgrasp/catkin_ws/src/passive_vision/camerainfo/";
const std::string topic_pointcloud = "pointcloud";
const int frame_width = 640;
const int frame_height = 480;
const unsigned int header_sequence_id = 0;
const std::string rgb_frame_id = "realsense";

// ROS Parameters
bool save_data = false;
bool display = false;
bool correct_depth = true;

// Global buffers for sensor data retrieval

float * cloud_buffer_pts = new float[frame_width * frame_height * 3];
uint8_t * cloud_buffer_rgb = new uint8_t[frame_width * frame_height * 3];
std::vector< std::vector<float> > color_cam_intrin;
std::map< std::string, std::vector<float> > map_ser_to_color_cam_intrin;
std::map< std::string, int > map_ser_to_idx;
std::vector<std::string> cam_serial_nums;
int device_count;
GLFWwindow * win;
GLFWwindow * win2;
rs::context ctx;
rs::device * active_dev;
rs::device * active_dev_bin0;
rs::device * active_dev_bin1;
rs::device * active_dev_bin2;
int active_dev_idx = 0;
std::string data_path;
std::vector<int> frame_idx;
image_transport::CameraPublisher rgb_image_pub;
image_transport::CameraPublisher rgb_image_pub_bin0;
image_transport::CameraPublisher rgb_image_pub_bin1;
image_transport::CameraPublisher rgb_image_pub_bin2;
image_transport::CameraPublisher depth_image_pub_bin0;
image_transport::CameraPublisher depth_image_pub_bin1;
image_transport::CameraPublisher depth_image_pub_bin2;
ros::Publisher pointcloud_pub;
std::map<std::string, ros::Publisher> map_pointcloud_pub;
std::map<std::string, std::vector<double> > map_pointcloud_xyzoffset;
sensor_msgs::CameraInfo rgb_camera_info;
uint8_t * rgb_cvmat_buffer;
uint8_t * rgb_cvmat_buffer_bin0;
uint8_t * rgb_cvmat_buffer_bin1;
uint8_t * rgb_cvmat_buffer_bin2;
cv::Mat rgb_cvmat;
cv::Mat rgb_cvmat_bin0;
cv::Mat rgb_cvmat_bin1;
cv::Mat rgb_cvmat_bin2;
pcl::PointCloud<PointType>::Ptr realsense_xyzrgb_cloud;
ros::Time header_time_stamp;

bool srv_capture(realsense_camera::snapshot::Request  &req,
                 realsense_camera::snapshot::Response &res);
void start_laset(int bin_id);



#endif