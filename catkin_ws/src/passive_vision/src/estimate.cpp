#include "ros/ros.h"
#include "realsense_camera/snapshot.h"
#include "passive_vision/state.h"
#include "robot_comm/robot_CartesianLog.h"
#include "vision_helper.h"
// #include <librealsense/rs.hpp>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>

#include <ctime>
#include <iostream>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <stdint.h>
#include <string>
#include <queue>
#include <time.h>

#include <sys/socket.h>    //torch_socket
#include <arpa/inet.h> //inet_addr
#include <netdb.h> //hostent

// #include <unistd.h>
#include <thread>
#include <mutex>


#include <map>

#include <Eigen/Geometry>

#include <stdio.h>
#include <time.h>




namespace pt = boost::property_tree;

using std::string;
using std::clock_t;

bool have_robot;
int im_width = 640;
int im_height = 480;
float * cam_K;

std::string torch_tcp_server_address = "127.0.0.1";
int torch_tcp_server_port = 5006;
struct sockaddr_in torch_tcp_server;
int torch_sock;
int torch_tcp_status;

std::string matlab_tcp_server_address = "127.0.0.1";
int matlab_tcp_server_port = 5007;
struct sockaddr_in matlab_tcp_server;
int matlab_sock;
int matlab_tcp_status;

std::string camera_serial_number;

std::vector<std::vector<std::string>> camera_serial_numbers = {{"","","",""},{"","","",""},{"","","",""},{"","","",""}};
// std::vector<std::vector<std::vector<float>>> cam2world_data = {{},{},{}};
std::vector<std::vector<Eigen::Matrix4f>> camera_pose = {{},{},{},{}};

std::vector<bool> bin_is_active = {false,false,false,false};
bool bin0_active = false;
bool bin1_active = false;
bool bin2_active = false;
bool bin3_active = false;
bool run_matlab = true;

std::vector<std::vector<std::vector<unsigned char>>> background_rgb = {{},{},{},{}};
std::vector<std::vector<std::vector<float>>> background_xyz = {{},{},{},{}};

std::vector<std::vector<std::vector<unsigned char>>> foreground_rgb = {{{},{}},{{},{}},{{},{}},{{},{}}};
std::vector<std::vector<std::vector<float>>> foreground_xyz = {{{},{}},{{},{}},{{},{}},{{},{}}};

std::vector<std::vector<float>> bin_position = {{},{},{},{}};

std::vector<std::vector<std::vector<float>>> camera_intrinsics = {{{},{}},{{},{}},{{},{}},{{},{}}};

std::vector<std::vector<float>> suction_predictions = {{},{},{},{}};
std::vector<std::vector<float>> suctionside_predictions = {{},{},{},{}};
std::vector<std::vector<float>> grasp_predictions = {{},{},{},{}};
std::vector<std::vector<float>> flush_grasp_predictions = {{},{},{},{}};

std::vector<std::vector<float>> height_maps = {{},{},{},{}};

std::vector<std::vector<std::string>> suction_object_list = {{},{},{},{}};
std::vector<std::vector<std::string>> suctionside_object_list = {{},{},{},{}};
std::vector<std::vector<std::string>> grasp_object_list = {{},{},{},{}};
std::vector<std::vector<std::string>> flush_grasp_object_list = {{},{},{},{}};

std::vector<std::vector<float>> suction_object_confidence = {{},{},{},{}};
std::vector<std::vector<float>> suctionside_object_confidence = {{},{},{},{}};
std::vector<std::vector<float>> grasp_object_confidence = {{},{},{},{}};
std::vector<std::vector<float>> flush_grasp_object_confidence = {{},{},{},{}};

std::vector<std::vector<std::string>> state_object_list = {{},{},{},{}};
std::vector<std::vector<std::string>> state_object_list_ontop = {{},{},{},{}};
std::vector<std::vector<float>> state_object_confidence = {{},{},{},{}};
std::vector<std::vector<float>> state_object_pose = {{},{},{},{}};
std::vector<std::vector<float>> state_object_visibility = {{},{},{},{}};

std::vector<std::string> heightmap_timestamps = {"","","",""};
std::vector<std::string> camera_timestamps = {"","","",""};
std::vector<std::string> state_timestamps = {"","","",""};
std::vector<std::string> suction_timestamps = {"","","",""};
std::vector<std::string> grasp_timestamps = {"","","",""};

ros::ServiceServer service_estimate;
std::map<std::string, ros::ServiceClient> realsense_client;
std::vector<std::string> machine_per_bin(4);
bool debug_mode;

std::string data_directory = "/home/mcube/arcdata/";
std::string rgraspdata_directory = "/home/mcube/rgraspdata/";
std::string rgrasp_directory = "/home/mcube/rgrasp/";
std::string camerainfo_directory = "/home/mcube/rgrasp/catkin_ws/src/passive_vision/camerainfo/";


std::vector<bool> save_debug_requested = {false,false,false,false};
std::vector<std::string> save_debug_paths = {"","","",""};
std::vector<std::string> save_debug_timestamps = {"","","",""};

std::vector<std::string> debug_messages = {"","","",""};

std::vector<bool> bin_is_dirty = {false,false,false,false};
std::vector<bool> bin_is_dirty_override = {false,false,false,false};

// std::vector<std::queue<std::string>> command_queues = {};

std::vector<std::queue<passive_vision::state::Request>> passive_vision_request_queue;
// std::queue<passive_vision::state::Response> passive_vision_response_queue;
std::vector<passive_vision::state::Response> passive_vision_responses;

std::vector<bool> bin_is_ready = {false,false,false,false};

int job_id_counter = 0;

std::mutex matlab_mutex;
std::mutex torch_mutex;
std::mutex camera_mutex;
std::mutex debug_mutex;
std::vector<std::mutex> service_mutex;
std::vector<std::mutex> save_mutex;

// Return Unix (Epoch) time
std::string GetUnixTime() {
    std::time_t unit_time = std::time(nullptr);
    std::stringstream ss;
    ss << unit_time;
    return ss.str();
}

const std::string currentDateTime() {
  time_t     now = time(0);
  struct tm  tstruct;
  char       buf[80];
  tstruct = *localtime(&now);
  // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
  // for more information about date/time format
  strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

  return buf;
}

void UpdateDebugMessage(int bin_id, std::string debug_message) {
  std::lock_guard<std::mutex> scoped_lock(debug_mutex);

  int num_spaces = 50 - debug_message.length();
  for (int i = 0; i < num_spaces; ++i)
    debug_message = debug_message + " ";
  debug_messages[bin_id] = debug_message;

  for (int i = 0; i < 4; ++i) {
    if (i != bin_id) {
      debug_messages[i] = "                                                  ";
    }
  }

  // Show debug messages
  std::cout << debug_messages[0] << " | " << debug_messages[1] << " | " << debug_messages[2] << " | " << debug_messages[3] << std::endl;
}

void UpdateBinState(passive_vision::state::Request req) {
  
  double totaltstart, tstart; 
  totaltstart = tic();
  tstart = tic();
  
  // Setup compression parameters for images
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//  compression_params.push_back(CV_IMWRITE_PXM_BINARY);
  compression_params.push_back(0);

  //print_toc(tstart, "TIMING 1");


  // Parse request update command and determine its type (1 2 3 4)
  tstart = tic();
  std::string update_command = req.update_command;
  std::istringstream update_command_ss(update_command);
  std::vector<std::string> parsed_command{std::istream_iterator<std::string>{update_command_ss},
                                          std::istream_iterator<std::string>{}};
  int bin_id = req.bin_id;
  int update_command_type = 0;
  if (parsed_command.size() == 2 && std::strcmp(parsed_command[1].c_str(),"hm") == 0) {
    std::cout << parsed_command[1].c_str() << std::endl;
    update_command_type = 1; // update hm
  } else if (parsed_command.size() == 3) {
    update_command_type = 2; // update hm sg
  } else if (parsed_command.size() == 4 && std::strcmp(parsed_command[1].c_str(),"hm") == 0) {
    update_command_type = 3; // update hm sg state xxx
  } else if (parsed_command.size() == 2 && std::strcmp(parsed_command[1].c_str(),"state") == 0) {
    update_command_type = 4; // update state xxx
  }
  passive_vision::state::Response res;
  //print_toc(tstart, "TIMING 2 Parse request");

  // Copy object state from previous state
  tstart = tic();
  res.state_object_list = passive_vision_responses[bin_id].state_object_list;
  res.state_object_list_ontop = passive_vision_responses[bin_id].state_object_list_ontop;
  res.state_object_pose = passive_vision_responses[bin_id].state_object_pose;
  res.state_object_confidence = passive_vision_responses[bin_id].state_object_confidence;
  res.state_object_visibility = passive_vision_responses[bin_id].state_object_visibility;

  UpdateDebugMessage(bin_id,"[MASTER] Recieved update command: " + update_command);
  //print_toc(tstart, "TIMING 3 Copy object state from previous state");
  // std::cout << update_command_type << std::endl;

  // *********************************************************************************
  // Get RGB-D image from first camera and create point cloud
  // *********************************************************************************

  tstart = tic();
  UpdateDebugMessage(bin_id,"[MASTER] Capturing images from first camera.");

  // Get RGB-D frame from RealSense camera ROS service
  realsense_camera::snapshot realsense_srv_capture1, realsense_srv_capture2;
  realsense_srv_capture1.request.camera_serial_number = camera_serial_numbers[bin_id][0];
  realsense_srv_capture2.request.camera_serial_number = camera_serial_numbers[bin_id][2];

  GetSnapshot(realsense_srv_capture1, realsense_srv_capture2, bin_id, realsense_client, machine_per_bin);

  foreground_rgb[bin_id][0] = realsense_srv_capture1.response.point_cloud_rgb;
  foreground_xyz[bin_id][0] = realsense_srv_capture1.response.point_cloud_xyz;

  print_toc(tstart, "TIMING 5 Get RGB-D frame");
  
  tstart = tic();
  // Return scene image file paths
  std::vector<std::string>file_paths_cam_0 = realsense_srv_capture1.response.file_paths;

  // Save input color image
  cv::Mat input1_color_mat_raw(im_height, im_width, CV_8UC3);
  for (size_t dy = 0; dy < im_height; ++dy)
      for (size_t dx = 0; dx < im_width; ++dx) {
          cv::Vec3b& bgr_value = input1_color_mat_raw.at<cv::Vec3b>(dy, dx);
          bgr_value[0] = realsense_srv_capture1.response.point_cloud_rgb[dy * im_width * 3 + dx * 3 + 2];
          bgr_value[1] = realsense_srv_capture1.response.point_cloud_rgb[dy * im_width * 3 + dx * 3 + 1];
          bgr_value[2] = realsense_srv_capture1.response.point_cloud_rgb[dy * im_width * 3 + dx * 3 + 0];
      }
  {
    std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
    imwrite(data_directory + "tmpdata/passive-vision-input." + std::to_string(bin_id) + ".0.color.png", input1_color_mat_raw, compression_params);
    //imwrite(rgraspdata_directory + "passive_vision_data/passive-vision-input."  +  currentDateTime() + "." + std::to_string(bin_id) + ".0.color.png", input1_color_mat_raw, compression_params);
  }
  print_toc(tstart, "TIMING 6 Save input color image");

  // Save aligned depth image to disk
  tstart = tic();
  float * input1_depth_buffer = new float[im_height * im_width];
  for (size_t dy = 0; dy < im_height; dy++)
      for (size_t dx = 0; dx < im_width; dx++)
          input1_depth_buffer[dy * im_width + dx] = foreground_xyz[bin_id][0][(dy * im_width + dx) * 3 + 2];
  {
    std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
    WriteDepth(data_directory + "tmpdata/passive-vision-input." + std::to_string(bin_id) + ".0.depth.png", input1_depth_buffer, im_height, im_width);
    //WriteDepth(rgraspdata_directory + "passive_vision_data/passive-vision-input."  +  currentDateTime() + "." + std::to_string(bin_id) + ".0.depth.png", input1_depth_buffer, im_height, im_width);
  }
  print_toc(tstart, "TIMING 7 Save aligned depth image to disk");
  // *********************************************************************************
  // Get RGB-D image from second camera and create point cloud
  // *********************************************************************************

  tstart = tic();
  UpdateDebugMessage(bin_id,"[MASTER] Capturing images from second camera.");

  foreground_rgb[bin_id][1] = realsense_srv_capture2.response.point_cloud_rgb;
  foreground_xyz[bin_id][1] = realsense_srv_capture2.response.point_cloud_xyz;

  // Save scene image file paths
  std::vector<std::string>file_paths_cam_1 = realsense_srv_capture2.response.file_paths;

  // Save input color image
  cv::Mat input2_color_mat_raw(im_height, im_width, CV_8UC3);
  for (size_t dy = 0; dy < im_height; ++dy)
      for (size_t dx = 0; dx < im_width; ++dx) {
          cv::Vec3b& bgr_value = input2_color_mat_raw.at<cv::Vec3b>(dy, dx);
          bgr_value[0] = realsense_srv_capture2.response.point_cloud_rgb[dy * im_width * 3 + dx * 3 + 2];
          bgr_value[1] = realsense_srv_capture2.response.point_cloud_rgb[dy * im_width * 3 + dx * 3 + 1];
          bgr_value[2] = realsense_srv_capture2.response.point_cloud_rgb[dy * im_width * 3 + dx * 3 + 0];
      }
  {
    std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
    imwrite(data_directory + "tmpdata/passive-vision-input." + std::to_string(bin_id) + ".1.color.png", input2_color_mat_raw, compression_params);
    //imwrite(rgraspdata_directory + "passive_vision_data/passive-vision-input."  +  currentDateTime() + "." + std::to_string(bin_id) + ".1.color.png", input2_color_mat_raw, compression_params);
  }
  print_toc(tstart, "TIMING 8 Save input color image");
  
  // Save aligned depth image to disk
  tstart = tic();
  float * input2_depth_buffer = new float[im_height * im_width];
  tstart = tic();
  for (size_t dy = 0; dy < im_height; dy++)
      for (size_t dx = 0; dx < im_width; dx++)
          input2_depth_buffer[dy * im_width + dx] = foreground_xyz[bin_id][1][(dy * im_width + dx) * 3 + 2]; 
  {
    std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
    WriteDepth(data_directory + "tmpdata/passive-vision-input." + std::to_string(bin_id) + ".1.depth.png", input2_depth_buffer, im_height, im_width);
    //WriteDepth(rgraspdata_directory + "passive_vision_data/passive-vision-input."  +  currentDateTime() + "." + std::to_string(bin_id) + ".1.depth.png", input2_depth_buffer, im_height, im_width);
  }
  print_toc(tstart, "TIMING 9 Save aligned depth");
  
  tstart = tic();
  delete[] input1_depth_buffer;
  delete[] input2_depth_buffer;
  //print_toc(tstart, "TIMING 10 delete memory");

  // *********************************************************************************
  // TCP Messages
  // *********************************************************************************

  if (update_command_type == 1 || update_command_type == 2 || update_command_type == 3) {
    tstart = tic();

    // Send TCP message to Matlab module to process height map
    std::string matlab_reply;
    std::string hm_matlab_tcp_message = "HM " + std::to_string(bin_id);
    UpdateDebugMessage(bin_id,"[MATLAB] Computing height map.");
    {
      std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
      std::lock_guard<std::mutex> matlab_scoped_lock(matlab_mutex); 
      matlab_tcp_status = send(matlab_sock, hm_matlab_tcp_message.c_str(), strlen(hm_matlab_tcp_message.c_str()), 0);

      //Receive a reply from the server
      char matlab_buffer[512];
      if (recv(matlab_sock, matlab_buffer, sizeof(matlab_buffer), 0) < 0) {
          std::cout << "Matlab TCP recv failed." << std::endl;
      }
      matlab_reply = matlab_buffer;
    }

    // Read binary file containing height map
    std::string height_map_output_file = data_directory + "tmpdata/passive-vision-height-map." + std::to_string(bin_id) + ".output.bin";
    FILE *fp = fopen(height_map_output_file.c_str(), "rb");
    float height_map_arr[300*200];
    int iret = fread((void*)height_map_arr, sizeof(float), 300*200, fp);
    std::vector<float> tmp_height_map(height_map_arr, height_map_arr + sizeof height_map_arr / sizeof height_map_arr[0]);
    fclose(fp);
    res.height_map = tmp_height_map;
    print_toc(tstart, "TIMING 11 MATLAB Height map");
  }

  if (update_command_type == 1) {
    passive_vision_responses[bin_id] = res;
    service_mutex[bin_id].unlock();
    return;
  }

  if (update_command_type == 2 || update_command_type == 3) {
  
    // Send TCP message to deep learning module
    tstart = tic();
    UpdateDebugMessage(bin_id,"[TORCH ] Computing deep network predictions.");
    {
      std::lock_guard<std::mutex> torch_scoped_lock(torch_mutex); 
      std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
      std::string torch_tcp_message = std::to_string(bin_id) + " GET / HTTP/1.1\r\n\r\n";
      torch_tcp_status = send(torch_sock,torch_tcp_message.c_str(),strlen(torch_tcp_message.c_str()),0);

      //Receive a reply from the server
      char torch_buffer[512];
      std::string torch_reply;
      if( recv(torch_sock , torch_buffer , sizeof(torch_buffer) , 0) < 0) {
          std::cout << "Torch TCP recv failed." << std::endl;
      }
      torch_reply = torch_buffer;
    }
    
    print_toc(tstart, "TIMING 11 Torch");
  }

  // Create TCP message for Matlab module
  std::string matlab_tcp_message = "";
  if (update_command_type == 2) {
    matlab_tcp_message = "NULL " + std::to_string(bin_id);
  } else if (update_command_type == 3) {
    matlab_tcp_message = req.state_command;
    // for (int i = 4; i < parsed_command.size(); ++i) {
    //   if (i == (parsed_command.size()-1))
    //     matlab_tcp_message = matlab_tcp_message + parsed_command[i];
    //   else
    //     matlab_tcp_message = matlab_tcp_message + parsed_command[i] + " ";
    // }
  } else if (update_command_type == 4) {
    matlab_tcp_message = "fast " + req.state_command;
    // for (int i = 2; i < parsed_command.size(); ++i) {
    //   if (i == (parsed_command.size()-1))
    //     matlab_tcp_message = matlab_tcp_message + parsed_command[i];
    //   else
    //     matlab_tcp_message = matlab_tcp_message + parsed_command[i] + " ";
    // }
  }
  UpdateDebugMessage(bin_id,"[MASTER] TCP message: " + matlab_tcp_message);

  if (update_command_type == 3 || update_command_type == 4) {

    // Save state command to text file
    {
      std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
      std::ofstream outFile;
      outFile.open(data_directory + "tmpdata/state-command.txt");
      outFile << std::string(req.state_command + "\n");
      outFile.close();
    }
  }

  if (update_command_type == 2 || update_command_type == 3 || update_command_type == 4) {

    tstart = tic();
    // std::string suction_output_file = data_directory + "tmpdata/passive-vision-suction." + std::to_string(bin_id) + ".output.bin";
    // std::string suction_objects_file = data_directory + "tmpdata/passive-vision-suction-objects." + std::to_string(bin_id) + ".output.txt";
    // std::string grasp_output_file = data_directory + "tmpdata/passive-vision-grasp." + std::to_string(bin_id) + ".output.bin";
    // std::string grasp_objects_file = data_directory + "tmpdata/passive-vision-grasp-objects." + std::to_string(bin_id) + ".output.txt";
    // std::string flush_grasp_output_file = data_directory + "tmpdata/passive-vision-flush-grasp." + std::to_string(bin_id) + ".output.bin";
    // std::string flush_grasp_objects_file = data_directory + "tmpdata/passive-vision-flush-grasp-objects." + std::to_string(bin_id) + ".output.txt";
    // system(std::string("rm " + suction_output_file).c_str());
    // system(std::string("rm " + suction_objects_file).c_str());
    // system(std::string("rm " + grasp_output_file).c_str());
    // system(std::string("rm " + grasp_objects_file).c_str());
    // system(std::string("rm " + flush_grasp_output_file).c_str());
    // system(std::string("rm " + flush_grasp_objects_file).c_str());

    // Send TCP message to Matlab module
    {
      std::string matlab_reply;
      std::lock_guard<std::mutex> matlab_scoped_lock(matlab_mutex); 
      std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
      UpdateDebugMessage(bin_id,"[MATLAB] Processing predictions.");
      matlab_tcp_status = send(matlab_sock, matlab_tcp_message.c_str(), strlen(matlab_tcp_message.c_str()), 0);

      //Receive a reply from the server
      char matlab_buffer[512];
      if (recv(matlab_sock, matlab_buffer, sizeof(matlab_buffer), 0) < 0) {
          std::cout << "Matlab TCP recv failed." << std::endl;
      }
      matlab_reply = matlab_buffer;
    }
    print_toc(tstart, "TIMING 12 Matlab state update and GS prediction");
  }

  // *********************************************************************************
  // Read binary file results
  // *********************************************************************************

  UpdateDebugMessage(bin_id,"[MASTER] Loading predictions.");

  //tstart = tic();
  if (update_command_type == 2 || update_command_type == 3) {
    
    // Read binary file containing suction predictions
    std::string suction_output_file = data_directory + "tmpdata/passive-vision-suction." + std::to_string(bin_id) + ".output.bin";
    FILE *fp = fopen(suction_output_file.c_str(), "rb");
    float num_suction_predictions;
    int iret = fread(&num_suction_predictions, sizeof(float), 1, fp);
    float suction_predictions_arr[(int)(8*num_suction_predictions)];
    iret = fread((void*)suction_predictions_arr, sizeof(float), 8*((int)num_suction_predictions), fp);
    // std::vector<float> tmp_suction_predictions(suction_predictions_arr, suction_predictions_arr + sizeof(suction_predictions_arr) / sizeof(suction_predictions_arr[0]));
    fclose(fp);

    std::ifstream suction_objects_file(data_directory + "tmpdata/passive-vision-suction-objects." + std::to_string(bin_id) + ".output.txt");
    suction_predictions[bin_id].clear();
    suction_object_confidence[bin_id].clear();
    suction_object_list[bin_id].clear();
    for (int i = 0; i < num_suction_predictions; ++i) {
      for (int j = 0; j < 7; ++j)
        suction_predictions[bin_id].push_back(suction_predictions_arr[i*8+j]);
      suction_object_confidence[bin_id].push_back(suction_predictions_arr[i*8+7]);
      std::string tmp_object_name;
      suction_objects_file >> tmp_object_name;
      suction_object_list[bin_id].push_back(tmp_object_name);
    }
    res.suction_points = suction_predictions[bin_id];
    res.suction_object_confidence = suction_object_confidence[bin_id];
    res.suction_object_list = suction_object_list[bin_id];

    if (bin_id == 0 || bin_id == 1) {

      // Read binary file containing suction side predictions
      std::string suctionside_output_file = data_directory + "tmpdata/passive-vision-suctionside." + std::to_string(bin_id) + ".output.bin";
      fp = fopen(suctionside_output_file.c_str(), "rb");
      float num_suctionside_predictions;
      iret = fread(&num_suctionside_predictions, sizeof(float), 1, fp);
      float suctionside_predictions_arr[(int)(8*num_suctionside_predictions)];
      iret = fread((void*)suctionside_predictions_arr, sizeof(float), 8*((int)num_suctionside_predictions), fp);
      fclose(fp);

      std::ifstream suctionside_objects_file(data_directory + "tmpdata/passive-vision-suctionside-objects." + std::to_string(bin_id) + ".output.txt");
      suctionside_predictions[bin_id].clear();
      suctionside_object_confidence[bin_id].clear();
      suctionside_object_list[bin_id].clear();
      for (int i = 0; i < num_suctionside_predictions; ++i) {
        for (int j = 0; j < 7; ++j)
          suctionside_predictions[bin_id].push_back(suctionside_predictions_arr[i*8+j]);
        suctionside_object_confidence[bin_id].push_back(suctionside_predictions_arr[i*8+7]);
        std::string tmp_object_name;
        suctionside_objects_file >> tmp_object_name;
        suctionside_object_list[bin_id].push_back(tmp_object_name);
      }
      res.suctionside_points = suctionside_predictions[bin_id];
      res.suctionside_object_confidence = suctionside_object_confidence[bin_id];
      res.suctionside_object_list = suctionside_object_list[bin_id];
    }
    
    // Read binary file containing grasp predictions
    std::string grasp_output_file = data_directory + "tmpdata/passive-vision-grasp." + std::to_string(bin_id) + ".output.bin";
    fp = fopen(grasp_output_file.c_str(), "rb");
    float num_grasp_predictions;
    iret = fread(&num_grasp_predictions, sizeof(float), 1, fp);
    float grasp_predictions_arr[(int)(13*num_grasp_predictions)];
    iret = fread((void*)grasp_predictions_arr, sizeof(float), 13*((int)num_grasp_predictions), fp);
    std::vector<float> tmp_grasp_predictions(grasp_predictions_arr, grasp_predictions_arr + sizeof grasp_predictions_arr / sizeof grasp_predictions_arr[0]);
    fclose(fp);

    std::ifstream grasp_objects_file(data_directory + "tmpdata/passive-vision-grasp-objects." + std::to_string(bin_id) + ".output.txt");
    grasp_predictions[bin_id].clear();
    grasp_object_confidence[bin_id].clear();
    grasp_object_list[bin_id].clear();
    for (int i = 0; i < num_grasp_predictions; ++i) {
      for (int j = 0; j < 12; ++j)
        grasp_predictions[bin_id].push_back(tmp_grasp_predictions[i*13+j]);
      grasp_object_confidence[bin_id].push_back(tmp_grasp_predictions[i*13+12]);
      std::string tmp_object_name;
      grasp_objects_file >> tmp_object_name;
      grasp_object_list[bin_id].push_back(tmp_object_name);
    }
    res.grasp_proposals = grasp_predictions[bin_id];
    res.grasp_object_confidence = grasp_object_confidence[bin_id];
    res.grasp_object_list = grasp_object_list[bin_id];

    // Read binary file containing flush grasp predictions
    std::string flush_grasp_output_file = data_directory + "tmpdata/passive-vision-flush-grasp." + std::to_string(bin_id) + ".output.bin";
    fp = fopen(flush_grasp_output_file.c_str(), "rb");
    float num_flush_grasp_predictions;
    iret = fread(&num_flush_grasp_predictions, sizeof(float), 1, fp);
    float flush_grasp_predictions_arr[(int)(7*num_flush_grasp_predictions)];
    iret = fread((void*)flush_grasp_predictions_arr, sizeof(float), 7*((int)num_flush_grasp_predictions), fp);
    std::vector<float> tmp_flush_grasp_predictions(flush_grasp_predictions_arr, flush_grasp_predictions_arr + sizeof flush_grasp_predictions_arr / sizeof flush_grasp_predictions_arr[0]);
    fclose(fp);

    std::ifstream flush_grasp_objects_file(data_directory + "tmpdata/passive-vision-flush-grasp-objects." + std::to_string(bin_id) + ".output.txt");
    flush_grasp_predictions[bin_id].clear();
    flush_grasp_object_confidence[bin_id].clear();
    flush_grasp_object_list[bin_id].clear();
    for (int i = 0; i < num_flush_grasp_predictions; ++i) {
      for (int j = 0; j < 6; ++j)
        flush_grasp_predictions[bin_id].push_back(tmp_flush_grasp_predictions[i*7+j]);
      flush_grasp_object_confidence[bin_id].push_back(tmp_flush_grasp_predictions[i*7+6]);
      std::string tmp_object_name;
      flush_grasp_objects_file >> tmp_object_name;
      flush_grasp_object_list[bin_id].push_back(tmp_object_name);
    }
    res.flush_grasp_proposals = flush_grasp_predictions[bin_id];
    res.flush_grasp_object_confidence = flush_grasp_object_confidence[bin_id];
    res.flush_grasp_object_list = flush_grasp_object_list[bin_id];

  }

  if (update_command_type == 3 || update_command_type == 4) {

    // Read binary file containing state
    std::string state_output_txt_path = data_directory + "tmpdata/passive-vision-state." + std::to_string(bin_id) + ".output.txt";
    std::string state_output_bin_path = data_directory + "tmpdata/passive-vision-state." + std::to_string(bin_id) + ".output.bin";
    
    state_object_list[bin_id].clear();
    state_object_list_ontop[bin_id].clear();
    state_object_pose[bin_id].clear();
    state_object_confidence[bin_id].clear();
    state_object_visibility[bin_id].clear();

    if (FileExists(state_output_txt_path) && FileExists(state_output_bin_path)) {

      std::ifstream state_output_txt_file;
      state_output_txt_file.open(state_output_txt_path);
      std::string tmp_word;
      int object_stack_count = 0;
      while (state_output_txt_file >> tmp_word) {
        // std::cout << tmp_word << std::endl;
        state_object_list_ontop[bin_id].push_back(tmp_word);
        if (object_stack_count == 0)
          state_object_list[bin_id].push_back(tmp_word);
        if (object_stack_count < 0)
          object_stack_count = std::stoi(tmp_word);
        else
          object_stack_count--; // obj1 3 obj2 obj4 obj7 obj3 2 obj5 obj6
      }
      state_output_txt_file.close();

      FILE * fp = fopen(state_output_bin_path.c_str(), "rb");
      float num_state_objects;
      int iret = fread(&num_state_objects, sizeof(float), 1, fp);
      float state_objects_info_arr[(int)(9*num_state_objects)];
      iret = fread((void*)state_objects_info_arr, sizeof(float), 9*((int)num_state_objects), fp);
      std::vector<float> tmp_state_objects_info(state_objects_info_arr, state_objects_info_arr + sizeof state_objects_info_arr / sizeof state_objects_info_arr[0]);
      fclose(fp);

      for (int i = 0; i < num_state_objects; ++i) {
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 1]);
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 2]);
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 3]); 
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 0]); // qw should be last
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 4]);
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 5]);
        state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 6]);
        state_object_confidence[bin_id].push_back(tmp_state_objects_info[9*i + 7]);
        state_object_visibility[bin_id].push_back(tmp_state_objects_info[9*i + 8]);
      }
      res.state_object_list = state_object_list[bin_id];
      res.state_object_list_ontop = state_object_list_ontop[bin_id];
      res.state_object_pose = state_object_pose[bin_id];
      res.state_object_confidence = state_object_confidence[bin_id];
      res.state_object_visibility = state_object_visibility[bin_id];
    }
  }

  //print_toc(tstart, "TIMING 13 Read result");
  
  //tstart = tic();
  passive_vision_responses[bin_id] = res;
  service_mutex[bin_id].unlock();

  // Request copy debug images
  std::string debug_data_path = data_directory + "loopdata/passive_vision/" + GetUnixTime() + "/";
  save_debug_paths[bin_id] = debug_data_path;
  save_debug_requested[bin_id] = true;
  //print_toc(tstart, "TIMING 14 Request copy debug images");
  print_toc(totaltstart, "TIMING Total");

}



// void CopyDebugImages(int bin_id,std::string debug_data_path) {
//   while (image_saving_locked[bin_id])
//     usleep(100 * 1000);

//   // Copy debug images
//   system(("mkdir -p " + debug_data_path).c_str());
//   system(std::string("cp " + data_directory + "tmpdata/*.png " + debug_data_path).c_str());
//   system(std::string("cp " + data_directory + "tmpdata/*.txt " + debug_data_path).c_str());
//   system(std::string("cp " + data_directory + "tmpdata/*.bin " + debug_data_path).c_str());
//   system(std::string("cp " + data_directory + "tmpdata/*.hdf5 " + debug_data_path).c_str());
//   system(std::string("cp " + data_directory + "tmpdata/*.mseg " + debug_data_path).c_str());
// }

// template <class T>
// void addVectorToJSON(pt::ptree & root, std::vector<T> const& row, std::string const& name) {
//     int n = row.size();
//     pt::ptree vector_node;
//     for (int i = 0; i < n; i++) {
//         // Create an unnamed value
//         pt::ptree cell;
//         cell.put_value(row[i]);
//         // Add the value to our row
//         vector_node.push_back(std::make_pair("", cell));
//     }
//     root.add_child(name, vector_node);
// }

// void DirtyBinCallback(const robot_comm::robot_CartesianLog::ConstPtr& msg) {
//   float robot_link6_x = (msg->x)/1000;
//   float robot_link6_y = (msg->y)/1000;
//   float robot_link6_z = (msg->z)/1000;

//   float tote_length = 0.61;
//   float tote_width = 0.37;

//   bool robot_in_home_position = std::abs(robot_link6_x - 0.905) < 0.01 &
//                                 std::abs(robot_link6_y + 0.001) < 0.01 &
//                                 std::abs(robot_link6_z - 0.740) < 0.01;
//   if (robot_in_home_position) {
//     for (int bin_id = 0; bin_id < 4; ++bin_id)
//       bin_is_dirty[bin_id] = false;
//     // std::cout << bin_is_dirty[0] << bin_is_dirty[1] << bin_is_dirty[2] << bin_is_dirty[3] << std::endl;
//     return;
//   }

//   for (int bin_id = 0; bin_id < 4; ++bin_id) {
//     // std::cout << std::abs(robot_link6_x - bin_position[bin_id][0]) << " " << std::abs(robot_link6_y - bin_position[bin_id][1]) <<std::endl;
//     bool robot_outside_bin = (std::abs(robot_link6_x - bin_position[bin_id][0]) > (tote_length/2)) |
//                              (std::abs(robot_link6_y - bin_position[bin_id][1]) > (tote_width/2));
//     bin_is_dirty[bin_id] = !robot_outside_bin;
//   }
//   // std::cout << bin_is_dirty[0] << bin_is_dirty[1] << bin_is_dirty[2] << bin_is_dirty[3] << std::endl;
// }

// Fetch background point cloud data
void ProcessBackground() {

  std::vector<int> compression_params;
//  compression_params.push_back(CV_IMWRITE_PXM_BINARY);
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(0);

  for (int bin_id = 0; bin_id < 4; ++bin_id) {
    if (bin_is_active[bin_id]) {

      background_rgb[bin_id].clear();
      background_xyz[bin_id].clear();

      // *********************************************************************************
      // Capture background RGB-D image for first camera
      // *********************************************************************************

      // Capture a point cloud of the empty tote (as background) from the RealSense camera
      realsense_camera::snapshot realsense_srv_capture1, realsense_srv_capture2;
      realsense_srv_capture1.request.camera_serial_number = camera_serial_numbers[bin_id][0];
      realsense_srv_capture2.request.camera_serial_number = camera_serial_numbers[bin_id][2];

      GetSnapshot(realsense_srv_capture1, realsense_srv_capture2, bin_id, realsense_client, machine_per_bin);

      background_rgb[bin_id].push_back(realsense_srv_capture1.response.point_cloud_rgb);
      background_xyz[bin_id].push_back(realsense_srv_capture1.response.point_cloud_xyz);
      camera_intrinsics[bin_id][0] = realsense_srv_capture1.response.color_camera_intrinsics;

      // Display background color image on debug window
      cv::Mat background1_color_mat(im_height, im_width, CV_8UC3);
      for (size_t dy = 0; dy < im_height; ++dy)
        for (size_t dx = 0; dx < im_width; ++dx) {
          cv::Vec3b& bgr_value = background1_color_mat.at<cv::Vec3b>(dy, dx);
          bgr_value[0] = background_rgb[bin_id][0][dy * im_width * 3 + dx * 3 + 2];
          bgr_value[1] = background_rgb[bin_id][0][dy * im_width * 3 + dx * 3 + 1];
          bgr_value[2] = background_rgb[bin_id][0][dy * im_width * 3 + dx * 3 + 0];
        }
      imwrite(data_directory + "tmpdata/passive-vision-background." + std::to_string(bin_id) + ".0.color.png", background1_color_mat, compression_params);

      float * background1_depth_buffer = new float[im_height * im_width];
      for (size_t dy = 0; dy < im_height; dy++)
        for (size_t dx = 0; dx < im_width; dx++) 
          background1_depth_buffer[dy * im_width + dx] = background_xyz[bin_id][0][(dy * im_width + dx) * 3 + 2];
      WriteDepth(data_directory + "tmpdata/passive-vision-background." + std::to_string(bin_id) + ".0.depth.png", background1_depth_buffer, im_height, im_width);

      // *********************************************************************************
      // Capture background RGB-D image for second camera
      // *********************************************************************************

      background_rgb[bin_id].push_back(realsense_srv_capture2.response.point_cloud_rgb);
      background_xyz[bin_id].push_back(realsense_srv_capture2.response.point_cloud_xyz);
      camera_intrinsics[bin_id][1] = realsense_srv_capture2.response.color_camera_intrinsics;

      // Display background color image on debug window
      cv::Mat background2_color_mat(im_height, im_width, CV_8UC3);
      for (size_t dy = 0; dy < im_height; ++dy)
        for (size_t dx = 0; dx < im_width; ++dx) {
          cv::Vec3b& bgr_value = background2_color_mat.at<cv::Vec3b>(dy, dx);
          bgr_value[0] = background_rgb[bin_id][1][dy * im_width * 3 + dx * 3 + 2];
          bgr_value[1] = background_rgb[bin_id][1][dy * im_width * 3 + dx * 3 + 1];
          bgr_value[2] = background_rgb[bin_id][1][dy * im_width * 3 + dx * 3 + 0];
        }
      imwrite(data_directory + "tmpdata/passive-vision-background." + std::to_string(bin_id) + ".1.color.png", background2_color_mat, compression_params);

      float * background2_depth_buffer = new float[im_height * im_width];
      for (size_t dy = 0; dy < im_height; dy++)
        for (size_t dx = 0; dx < im_width; dx++) 
          background2_depth_buffer[dy * im_width + dx] = background_xyz[bin_id][1][(dy * im_width + dx) * 3 + 2];
      WriteDepth(data_directory + "tmpdata/passive-vision-background." + std::to_string(bin_id) + ".1.depth.png", background2_depth_buffer, im_height, im_width);

      // *********************************************************************************
      // Save camera intrinsics and pose
      // *********************************************************************************

      // Save camera intrinsics of color sensor
      std::string cam_intrinsics_file = data_directory + "tmpdata/passive-vision-camera." + std::to_string(bin_id) + ".0.intrinsics.txt";
      FILE *fp = fopen(cam_intrinsics_file.c_str(), "w");
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", camera_intrinsics[bin_id][0][0], camera_intrinsics[bin_id][0][1], camera_intrinsics[bin_id][0][2]);
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", camera_intrinsics[bin_id][0][3], camera_intrinsics[bin_id][0][4], camera_intrinsics[bin_id][0][5]);
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n\n", camera_intrinsics[bin_id][0][6], camera_intrinsics[bin_id][0][7], camera_intrinsics[bin_id][0][8]);
      fclose(fp);
      cam_intrinsics_file = data_directory + "tmpdata/passive-vision-camera." + std::to_string(bin_id) + ".1.intrinsics.txt";
      fp = fopen(cam_intrinsics_file.c_str(), "w");
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", camera_intrinsics[bin_id][1][0], camera_intrinsics[bin_id][1][1], camera_intrinsics[bin_id][1][2]);
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", camera_intrinsics[bin_id][1][3], camera_intrinsics[bin_id][1][4], camera_intrinsics[bin_id][1][5]);
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n\n", camera_intrinsics[bin_id][1][6], camera_intrinsics[bin_id][1][7], camera_intrinsics[bin_id][1][8]);
      fclose(fp);

      std::string cam_pose_file = data_directory + "tmpdata/passive-vision-camera." + std::to_string(bin_id) + ".0.pose.txt";
      fp = fopen(cam_pose_file.c_str(), "w");
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", camera_pose[bin_id][0](0,0), camera_pose[bin_id][0](0,1), camera_pose[bin_id][0](0,2), camera_pose[bin_id][0](0,3));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", camera_pose[bin_id][0](1,0), camera_pose[bin_id][0](1,1), camera_pose[bin_id][0](1,2), camera_pose[bin_id][0](1,3));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", camera_pose[bin_id][0](2,0), camera_pose[bin_id][0](2,1), camera_pose[bin_id][0](2,2), camera_pose[bin_id][0](2,3));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n\n", camera_pose[bin_id][0](3,0), camera_pose[bin_id][0](3,1), camera_pose[bin_id][0](3,2), camera_pose[bin_id][0](3,3));
      fclose(fp);
      cam_pose_file = data_directory + "tmpdata/passive-vision-camera." + std::to_string(bin_id) + ".1.pose.txt";
      fp = fopen(cam_pose_file.c_str(), "w");
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", camera_pose[bin_id][2](0,0), camera_pose[bin_id][2](0,1), camera_pose[bin_id][2](0,2), camera_pose[bin_id][2](0,3));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", camera_pose[bin_id][2](1,0), camera_pose[bin_id][2](1,1), camera_pose[bin_id][2](1,2), camera_pose[bin_id][2](1,3));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n", camera_pose[bin_id][2](2,0), camera_pose[bin_id][2](2,1), camera_pose[bin_id][2](2,2), camera_pose[bin_id][2](2,3));
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t %15.8e\t\n\n", camera_pose[bin_id][2](3,0), camera_pose[bin_id][2](3,1), camera_pose[bin_id][2](3,2), camera_pose[bin_id][2](3,3));
      fclose(fp);


      delete[] background1_depth_buffer;
      delete[] background2_depth_buffer;
    }
  }


}

// Service: call passive vision to get latest state, suction points, grasp proposals 
bool SrvEstimate(passive_vision::state::Request  &req,
                 passive_vision::state::Response &res) {

  std::string update_command = req.update_command;
  int bin_id = req.bin_id;

  // Check if command is a request
  if (std::strcmp(req.update_command.substr(0,7).c_str(),"request") == 0) {
    // while (!bin_is_ready[bin_id])
    //   usleep(10*1000);

    // {
    //   std::lock_guard<std::mutex> service_scoped_lock(service_mutex[bin_id]);

    std::cout << "Acquiring lock" << std::endl;
    service_mutex[bin_id].lock();
    std::cout << "Copying values" << std::endl;
    res = passive_vision_responses[bin_id];
    std::cout << "Freeing lock" << std::endl;
    service_mutex[bin_id].unlock();

    // }

  } else if (std::strcmp(req.update_command.substr(0,7).c_str(),"restart") == 0) {

    // Delete old tmpdata files
    system(std::string("rm " + data_directory + "tmpdata/*.png").c_str());
    system(std::string("rm " + data_directory + "tmpdata/*.txt").c_str());
    system(std::string("rm " + data_directory + "tmpdata/*.bin").c_str());
    system(std::string("rm " + data_directory + "tmpdata/*.hdf5").c_str());
    system(std::string("rm " + data_directory + "tmpdata/*.mseg").c_str());
    system(std::string("rm " + data_directory + "tmpdata/*.lock").c_str());

    ProcessBackground();
    std::cout << "[MASTER]: Captured background." << std::endl;

    // Send TCP message to Matlab module
    std::string matlab_tcp_message = "- restart";
    std::string matlab_reply;
    std::lock_guard<std::mutex> matlab_scoped_lock(matlab_mutex); 
    std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
    matlab_tcp_status = send(matlab_sock, matlab_tcp_message.c_str(), strlen(matlab_tcp_message.c_str()), 0);

    //Receive a reply from the server
    char matlab_buffer[512];
    if (recv(matlab_sock, matlab_buffer, sizeof(matlab_buffer), 0) < 0) {
        std::cout << "Matlab TCP recv failed." << std::endl;
    }
    matlab_reply = matlab_buffer;

  // If command is an update, add to queue and return empty message with job ID
  } else {
    passive_vision::state::Request reqCopy = req;

    service_mutex[bin_id].lock();
    passive_vision_request_queue[reqCopy.bin_id].push(reqCopy);
    
    // std::cout << "got here" << std::endl;
    // std::thread updateThread(UpdateBinState, reqCopy);
    // res.job_id = reqCopy.job_id;
  }
  
  return true;
}

void ProfileSrvCall(){
  std::cout << "Starting profile" << std::endl;
  struct timespec start, finish;
  double elapsed;
  clock_gettime(CLOCK_MONOTONIC, &start);

  passive_vision::state::Request  req;
  req.update_command = "update hm sg";
  req.state_command = "";
  req.bin_id = 0;
  passive_vision::state::Response res;

  for (int i = 0; i < 20; ++i) {
    std::cout << "Call update" << std::endl;
    UpdateBinState(req);
  }

  clock_gettime(CLOCK_MONOTONIC, &finish);
  elapsed = (finish.tv_sec - start.tv_sec) + (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
  std::cout << "Elapsed time: " << elapsed << std::endl;
}

void LoopBinUpdates(int bin_id) {
  while (true) {
    if (!passive_vision_request_queue[bin_id].empty()) {
      int bin_update_start_timestamp = std::stoi(GetUnixTime());
      passive_vision::state::Request tmp_req = passive_vision_request_queue[bin_id].front();
      passive_vision_request_queue[bin_id].pop();
      UpdateBinState(tmp_req);
      
      char buffer[50];
      int bin_update_end_timestamp = std::stoi(GetUnixTime());
      int iret = sprintf(buffer, "[MASTER] Elapsed time is %d seconds.", bin_update_end_timestamp - bin_update_start_timestamp);
      std::string str_buffer(buffer);
      UpdateDebugMessage(bin_id,str_buffer);
    } else {
      usleep(10 * 1000);
    }
  }
}
//   int bin_update_start_timestamp = std::stoi(GetUnixTime());
//   while (bin_is_active[bin_id]) {
//     if (!bin_is_dirty[bin_id] && !bin_is_dirty_override[bin_id]) {
//       UpdateBinState(bin_id);
//       char buffer[50];
//       int iret = sprintf(buffer, "[MASTER] Elapsed time is %d seconds.", bin_update_end_timestamp - bin_update_start_timestamp);
//       std::string str_buffer(buffer);
//       UpdateDebugMessage(bin_id,str_buffer);
//     }
//   }
// }

void SystemCallTorch() {
  system(std::string("cd " + rgrasp_directory + "catkin_ws/src/passive_vision/src; th suction.lua").c_str());
}

void SystemCallMatlab() {
  system(std::string("cd " + rgrasp_directory + "catkin_ws/src/passive_vision/src; matlab_light -nodisplay -nodesktop -noFigureWindows -r 'main;'").c_str());
}

// void AsyncMatlabDebug() {
//   while (true) {
//     for (int bin_id = 0; bin_id < 4; ++bin_id) {
//       if (bin_is_active[bin_id] && save_debug_requested[bin_id]) {
//         save_debug_requested[bin_id] = false;
//       }
//     }
//   }
//   system(std::string("cd " + rgrasp_directory + "catkin_ws/src/passive_vision/src; matlab -nodisplay -nodesktop -r 'getDebugImages;'").c_str());
// }

void AsyncSaveDebug() {
  while (true) {
    for (int bin_id = 0; bin_id < 4; ++bin_id) {
      if (bin_is_active[bin_id] && save_debug_requested[bin_id]) {
        // while (image_saving_locked[bin_id])
        //   usleep(100 * 1000);
        {
          std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 
          system(("mkdir -p " + save_debug_paths[bin_id]).c_str());
          system(std::string("cp " + data_directory + "tmpdata/*.txt " + save_debug_paths[bin_id]).c_str());
          system(std::string("cp " + data_directory + "tmpdata/*.bin " + save_debug_paths[bin_id]).c_str());
          system(std::string("cp " + data_directory + "tmpdata/*.hdf5 " + save_debug_paths[bin_id]).c_str());
          system(std::string("cp " + data_directory + "tmpdata/*.mseg " + save_debug_paths[bin_id]).c_str());
          system(std::string("cp " + data_directory + "tmpdata/*.png " + save_debug_paths[bin_id]).c_str());
          // system(std::string("cp " + data_directory + "tmpdata/*.mat " + save_debug_paths[bin_id]).c_str());
          UpdateDebugMessage(bin_id,"[MASTER] Saved images.");
          save_debug_requested[bin_id] = false;
        }
      }
      else{
        usleep(10 * 1000);
      }
    }
  }
}

// // Call Torch for deep suction prediction
// void AsyncTorchTCP() {
//   while (true) {
//     for (int bin_id = 0; bin_id < 4; ++bin_id) {
//       if (bin_is_active[bin_id] && torch_requested[bin_id]) {
//         image_saving_locked[bin_id] = true;
//         std::string torch_tcp_message = std::to_string(bin_id) + " GET / HTTP/1.1\r\n\r\n";
//         torch_tcp_status = send(torch_sock,torch_tcp_message.c_str(),strlen(torch_tcp_message.c_str()),0);

//         //Receive a reply from the server
//         char torch_buffer[512];
//         std::string torch_reply;
//         if( recv(torch_sock , torch_buffer , sizeof(torch_buffer) , 0) < 0) {
//             std::cout << "Torch TCP recv failed." << std::endl;
//         }
//         std::cout << "finished torch reply" << std::endl;
//         torch_reply = torch_buffer;
//         torch_requested[bin_id] = false;
//         image_saving_locked[bin_id] = false;
//       }
//     }
//   }
// }

int main(int argc, char **argv) {
  // Setup ROS service server
  ros::init(argc, argv, "passive_vision");

  ros::NodeHandle n;
  ros::param::get("/have_robot", have_robot);
  service_estimate = n.advertiseService("passive_vision/estimate", SrvEstimate);
  realsense_client["arc_1"] = n.serviceClient<realsense_camera::snapshot>("/arc_1/realsense_camera/capture");
  // realsense_client["arc_1"] = n.serviceClient<realsense_camera::snapshot>("/arc_1/realsense_camera/capture");

  // Setup ROS subscriber for robot link 6
  // ros::Subscriber robotCartesianSub = n.subscribe("robot1_CartesianLog", 1, DirtyBinCallback);

  // Get parameters from ROS call
  ros::NodeHandle priv_nh("~");
  priv_nh.param("bin0_active", bin0_active, false); 
  priv_nh.param("bin1_active", bin1_active, false);
  priv_nh.param("bin2_active", bin2_active, false);
  priv_nh.param("bin3_active", bin3_active, false);
  priv_nh.param("run_matlab", run_matlab, true);
  // std::cout <<  bin0_active << bin1_active << bin2_active << bin3_active << std::endl;
  bin_is_active[0] = bin0_active;
  bin_is_active[1] = bin1_active;
  bin_is_active[2] = bin2_active;
  bin_is_active[3] = bin3_active;

  // Initialize 4 mutexes to handle service calls
  std::vector<std::mutex> service_mutex_list(4);
  service_mutex.swap(service_mutex_list);

  // Initialize 4 mutexes to handle image saving
  std::vector<std::mutex> save_mutex_list(4);
  save_mutex.swap(save_mutex_list);

  FILE * fp = fopen(std::string(camerainfo_directory + "bins.txt").c_str(),"r");
  int bin_id = -1;
  for (int i = 0; i < 4; ++i) {
    int successful = fscanf(fp, "bin %d\n", &bin_id);
    float bin_pose_x;
    float bin_pose_y;
    float bin_pose_z;
    successful = fscanf(fp, "pose %f %f %f\n", &bin_pose_x, &bin_pose_y, &bin_pose_z);
    bin_position[bin_id].push_back(bin_pose_x);
    bin_position[bin_id].push_back(bin_pose_y);
    bin_position[bin_id].push_back(bin_pose_z);

    char * top_far_camera_id = new char[12];
    char * bottom_far_camera_id = new char[12];
    char * top_near_camera_id = new char[12];
    char * bottom_near_camera_id = new char[12];
    successful = fscanf(fp, "tf %s\n", top_far_camera_id);
    successful = fscanf(fp, "bf %s\n", bottom_far_camera_id);
    successful = fscanf(fp, "tn %s\n", top_near_camera_id);
    successful = fscanf(fp, "bn %s\n", bottom_near_camera_id);
    std::string bin("/camera/bin" + std::to_string(bin_id));
    ros::param::get(std::string(bin + "_passive_far/serial"), camera_serial_numbers[bin_id][0]);
    ros::param::get(std::string(bin + "_active_far/serial"), camera_serial_numbers[bin_id][1]);
    ros::param::get(std::string(bin + "_passive_near/serial"), camera_serial_numbers[bin_id][2]);
    ros::param::get(std::string(bin + "_active_near/serial"), camera_serial_numbers[bin_id][3]);
    std::cout << camera_serial_numbers[bin_id][0] << " " << camera_serial_numbers[bin_id][1] << " " << camera_serial_numbers[bin_id][2] << " " << camera_serial_numbers[bin_id][3] << std::endl;
    ros::param::get(std::string(bin + "_active_near/machine"), machine_per_bin[bin_id]);
//    camera_serial_numbers[bin_id][0] = std::string(top_far_camera_id);
//    camera_serial_numbers[bin_id][1] = std::string(bottom_far_camera_id);
//    camera_serial_numbers[bin_id][2] = std::string(top_near_camera_id);
//    camera_serial_numbers[bin_id][3] = std::string(bottom_near_camera_id);
  }
  fclose(fp);

  // Fetch camera poses
  for (int bin_id = 0; bin_id < 4; ++bin_id) {
    Eigen::Matrix4f identity_mat = Eigen::Matrix4f::Identity(4,4);
    for (int i = 0; i < 4; ++i)
      camera_pose[bin_id].push_back(identity_mat);
    if (bin_is_active[bin_id])
      std::cout << "[MASTER]: Bin #" << bin_id << " is ACTIVE:" << std::endl;
    else
      std::cout << "[MASTER]: Bin #" << bin_id << " is INACTIVE." << std::endl;
    for (int i = 0; i < 4; ++i)
      if (bin_is_active[bin_id]) {
        std::string camera_extrinsics_file = camerainfo_directory + rtrim(camera_serial_numbers[bin_id][i]) + ".pose.txt";
        // std::cout << camera_extrinsics_file << std::endl;
        if (FileExists(camera_extrinsics_file)) {
          std::vector<float> cam2world_vec = LoadMatrixFromFile(camera_extrinsics_file,4,4);
          Eigen::Matrix4f cam2world_mat;
          cam2world_mat << cam2world_vec[0], cam2world_vec[1], cam2world_vec[2], cam2world_vec[3],
                           cam2world_vec[4], cam2world_vec[5], cam2world_vec[6], cam2world_vec[7],
                           cam2world_vec[8], cam2world_vec[9], cam2world_vec[10], cam2world_vec[11],
                           cam2world_vec[12], cam2world_vec[13], cam2world_vec[14], cam2world_vec[15];
          camera_pose[bin_id][i] = cam2world_mat;
          std::cout << "[MASTER]:     Camera: " << camera_serial_numbers[bin_id][i] << std::endl;
        }
      }
  }

  for (int bin_id = 0; bin_id < 4; ++bin_id) {
    std::queue<passive_vision::state::Request> tmp_request_queue;
    passive_vision_request_queue.push_back(tmp_request_queue);
    passive_vision::state::Response tmp_res;
    passive_vision_responses.push_back(tmp_res);
  }

  // Start Torch and Matlab modules in parallel
  std::thread torchThread(SystemCallTorch);
  std::shared_ptr<std::thread> pmatlabThread;
  
  if(run_matlab)
    pmatlabThread = std::make_shared<std::thread>(SystemCallMatlab);
  usleep(5000*1000);

  // Setup socket connection to call Torch module
  torch_sock = socket(AF_INET,SOCK_STREAM,0);
  torch_tcp_server.sin_addr.s_addr = inet_addr(torch_tcp_server_address.c_str());
  torch_tcp_server.sin_family = AF_INET;
  torch_tcp_server.sin_port = htons(torch_tcp_server_port);
  
  while(true){
    torch_tcp_status = connect(torch_sock,(struct sockaddr *)&torch_tcp_server ,sizeof(torch_tcp_server));
    if (torch_tcp_status < 0)
      std::cout << "[MASTER]: ERROR: TCP connection to Torch module failed. Retry" << std::endl;
    else{
      std::cout << "[MASTER]: Successfully established TCP connection to Torch module." << std::endl;
      break;
    }
    usleep(1000*1000);
  }

  // Setup socket connection to call Matlab modules
  matlab_sock = socket(AF_INET,SOCK_STREAM,0);
  matlab_tcp_server.sin_addr.s_addr = inet_addr(matlab_tcp_server_address.c_str());
  matlab_tcp_server.sin_family = AF_INET;
  matlab_tcp_server.sin_port = htons(matlab_tcp_server_port);
  
  while(true){
    matlab_tcp_status = connect(matlab_sock,(struct sockaddr *)&matlab_tcp_server ,sizeof(matlab_tcp_server));
    if (matlab_tcp_status < 0)
      std::cout << "[MASTER]: ERROR: TCP connection to Matlab module failed. Retry" << std::endl;
    else{
      std::cout << "[MASTER]: Successfully established TCP connection to Matlab module." << std::endl;
      break;
    }
    usleep(1000*1000);
  }

  // Check for existing background data and load it
  bool can_load_prev_state = true;
  for (int bin_id = 0; bin_id < 4; ++bin_id) {
    if(bin_is_active[bin_id]){
      for (int camera_idx = 0; camera_idx < 2; ++camera_idx) {
        std::string backgroundColorImgFile = data_directory + "tmpdata/passive-vision-background." + std::to_string(bin_id) + "." + std::to_string(camera_idx) + ".color.png";
        std::string backgroundDepthImgFile = data_directory + "tmpdata/passive-vision-background." + std::to_string(bin_id) + "." + std::to_string(camera_idx) + ".depth.png";
        can_load_prev_state = can_load_prev_state && FileExists(backgroundColorImgFile) && FileExists(backgroundDepthImgFile);
      }
    }
  }
  can_load_prev_state = can_load_prev_state && FileExists(data_directory+"tmpdata/allBinsData.mat");
  if (can_load_prev_state) {
    std::cout << "[MASTER]: Background files found! Re-loading previous state." << std::endl;
    usleep(1000*1000);

    // Send TCP message to Matlab module
    std::string matlab_tcp_message = "- loadprev";
    std::string matlab_reply;
    std::lock_guard<std::mutex> matlab_scoped_lock(matlab_mutex); 
    std::lock_guard<std::mutex> save_scoped_lock(save_mutex[bin_id]); 

    matlab_tcp_status = send(matlab_sock, matlab_tcp_message.c_str(), strlen(matlab_tcp_message.c_str()), 0);

    //Receive a reply from the server
    char matlab_buffer[512];
    if (recv(matlab_sock, matlab_buffer, sizeof(matlab_buffer), 0) < 0) {
        std::cout << "Matlab TCP recv failed." << std::endl;
    }
    matlab_reply = matlab_buffer;


    for (int bin_id = 0; bin_id < 4; ++bin_id) {
      if(bin_is_active[bin_id]){

        passive_vision::state::Response res;
        std::cout << "[MASTER]: Loading state for bin " + std::to_string(bin_id) + "." << std::endl;

        // Read binary file containing state
        std::string state_output_txt_path = data_directory + "tmpdata/passive-vision-state." + std::to_string(bin_id) + ".output.txt";
        std::string state_output_bin_path = data_directory + "tmpdata/passive-vision-state." + std::to_string(bin_id) + ".output.bin";
        
        state_object_list[bin_id].clear();
        state_object_list_ontop[bin_id].clear();
        state_object_pose[bin_id].clear();
        state_object_confidence[bin_id].clear();
        state_object_visibility[bin_id].clear();

        if (FileExists(state_output_txt_path) && FileExists(state_output_bin_path)) {

          std::ifstream state_output_txt_file;
          state_output_txt_file.open(state_output_txt_path);
          std::string tmp_word;
          int object_stack_count = 0;
          while (state_output_txt_file >> tmp_word) {
            // std::cout << tmp_word << std::endl;
            state_object_list_ontop[bin_id].push_back(tmp_word);
            if (object_stack_count == 0)
              state_object_list[bin_id].push_back(tmp_word);
            if (object_stack_count < 0)
              object_stack_count = std::stoi(tmp_word);
            else
              object_stack_count--; // obj1 3 obj2 obj4 obj7 obj3 2 obj5 obj6
          }
          state_output_txt_file.close();

          FILE * fp = fopen(state_output_bin_path.c_str(), "rb");
          float num_state_objects;
          int iret = fread(&num_state_objects, sizeof(float), 1, fp);
          float state_objects_info_arr[(int)(9*num_state_objects)];
          iret = fread((void*)state_objects_info_arr, sizeof(float), 9*((int)num_state_objects), fp);
          std::vector<float> tmp_state_objects_info(state_objects_info_arr, state_objects_info_arr + sizeof state_objects_info_arr / sizeof state_objects_info_arr[0]);
          fclose(fp);

          for (int i = 0; i < num_state_objects; ++i) {
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 1]);
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 2]);
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 3]); 
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 0]); // qw should be last
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 4]);
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 5]);
            state_object_pose[bin_id].push_back(tmp_state_objects_info[9*i + 6]);
            state_object_confidence[bin_id].push_back(tmp_state_objects_info[9*i + 7]);
            state_object_visibility[bin_id].push_back(tmp_state_objects_info[9*i + 8]);
          }
          res.state_object_list = state_object_list[bin_id];
          res.state_object_list_ontop = state_object_list_ontop[bin_id];
          res.state_object_pose = state_object_pose[bin_id];
          res.state_object_confidence = state_object_confidence[bin_id];
          res.state_object_visibility = state_object_visibility[bin_id];
        }

        passive_vision_responses[bin_id] = res;
      }
    }

  } else {
    std::cout << "[MASTER]: Failed to load previous state. Restarting as normal." << std::endl;

    // Delete old tmpdata files
    int successful = system(std::string("rm " + data_directory + "tmpdata/*.png").c_str());
    successful = system(std::string("rm " + data_directory + "tmpdata/*.txt").c_str());
    successful = system(std::string("rm " + data_directory + "tmpdata/*.bin").c_str());
    successful = system(std::string("rm " + data_directory + "tmpdata/*.hdf5").c_str());
    successful = system(std::string("rm " + data_directory + "tmpdata/*.mseg").c_str());
    successful = system(std::string("rm " + data_directory + "tmpdata/*.lock").c_str());

    ProcessBackground();
    std::cout << "[MASTER]: Captured background." << std::endl;
  }


//  ProfileSrvCall();
//  return 0;

  //std::thread asyncTorchThread(AsyncTorchTCP);
  //std::thread asyncMatlabThread(AsyncMatlabDebug);
  std::thread asyncDebugThread(AsyncSaveDebug);

  std::thread loopThread0(LoopBinUpdates, 0);
  std::thread loopThread1(LoopBinUpdates, 1);
  std::thread loopThread2(LoopBinUpdates, 2);
  std::thread loopThread3(LoopBinUpdates, 3);
  std::cout << "[MASTER]: Ready." << std::endl;

  // std::thread loopThread(LoopBinUpdates);
  
  ros::spin();
  return 0;
}
