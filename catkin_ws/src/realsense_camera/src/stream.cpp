#include "stream.h"
#include <ctime>
#include <string>
#include <stdio.h>
#include <time.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <boost/algorithm/string.hpp>

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

void WriteDepth(const std::string &depth_file, float * depth_values, int frame_height, int frame_width) {
  cv::Mat depth_mat(frame_height, frame_width, CV_16UC1);
  for (size_t y = 0; y < frame_height; y++)
    for (size_t x = 0; x < frame_width; x++) {
      unsigned short depth_short = (unsigned short)(depth_values[y * frame_width + x] * 10000);
      depth_mat.at<unsigned short>(y, x) = depth_short;
    }
  std::vector<int> compression_params;
//  compression_params.push_back(CV_IMWRITE_PXM_BINARY);
  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
  compression_params.push_back(9);
  cv::imwrite(depth_file, depth_mat, compression_params);
}

void publish_depth(const uint16_t * depth_values, int frame_height, int frame_width, image_transport::CameraPublisher _depth_image_pub, int counter, std::string serial) {
  sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

  cv::Mat depth_mat(frame_height, frame_width, CV_16UC1);
//  std::cout<<depth_mat<<std::endl;
  std_msgs::Float32MultiArray arr;
  for (size_t y = 0; y < frame_height; y++)
    for (size_t x = 0; x < frame_width; x++) {
      unsigned short depth_short = (unsigned short)(depth_values[y * frame_width + x] * 1);
      depth_mat.at<unsigned short>(y, x) = depth_short;
      arr.data.push_back(depth_short);
//      std::cout<<depth_short<<std::endl;
    }
////, ros::Publisher pub_depth_arr
//  pub_depth_arr.publish(arr);
//
//  std::vector<int> compression_params;
//  compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//  compression_params.push_back(9);
//  std::cout<<"publish depth"<<std::endl;
//  cv::imwrite(data_directory + "tmpdata/passive-vision-input2 " +  currentDateTime() + std::to_string(counter) +".1.depth.png", depth_mat, compression_params);

  //publish to ros image topic
  cv_bridge::CvImage img_bridge;
  sensor_msgs::Image img_msg; // >> message to be sent
  std_msgs::Header header; // empty header
  header.seq = header_sequence_id; // user defined counter
  header.stamp = header_time_stamp; // time
  img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, depth_mat);
  img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
  rgb_camera_info.header.frame_id = rgb_frame_id + "_" + serial;
  _depth_image_pub.publish(img_msg, rgb_camera_info); // ros::Publisher pub_img = node.advertise<sensor_msgs
}

void publish_rgb_image_msg(cv::Mat& rgb_mat, std::string serial, image_transport::CameraPublisher _rgb_image_pub)
{
  sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

  rgb_img->header.seq = header_sequence_id;
  rgb_img->header.stamp = header_time_stamp;
  rgb_img->header.frame_id = rgb_frame_id + "_" + serial;

  rgb_img->width = rgb_mat.cols;
  rgb_img->height = rgb_mat.rows;

  rgb_img->encoding = sensor_msgs::image_encodings::RGB8;
  rgb_img->is_bigendian = 0;

  int step = sizeof(unsigned char) * 3 * rgb_img->width;
  int size = step * rgb_img->height;
  rgb_img->step = step;
  rgb_img->data.resize(size);
  memcpy(&(rgb_img->data[0]), rgb_mat.data, size);


  rgb_camera_info.header.frame_id = rgb_frame_id + "_" + serial;
  rgb_camera_info.header.stamp = header_time_stamp;
  rgb_camera_info.header.seq = header_sequence_id;

  std::vector<float> intrin = map_ser_to_color_cam_intrin[serial];

  float fx = intrin[0];
  float fy = intrin[4];
  float ppx = intrin[2];
  float ppy = intrin[5];
  float coeffs[] = {0.0,0.0,0.0,0.0,0.0};

  setArrayFromScalars(rgb_camera_info.K, fx, 0.0f,        ppx,
                      0.0f,       fy,  ppy,
                      0.0f,       0.0f,        1.0f);


  setArrayFromScalars(rgb_camera_info.R, 1.0f, 0.0f,        0.0f,
                      0.0f,       1.0f,  0.0f,
                      0.0f,       0.0f,        1.0f);

  setArrayFromScalars(rgb_camera_info.P, fx,    0.0f,        ppx, 0.0f,
                      0.0f,          fy,  ppy, 0.0f,
                      0.0f,          0.0f,        1.0f, 0.0f);
  rgb_camera_info.distortion_model = "plumb_bob";
  for(int i=0; i<5; i++)
    rgb_camera_info.D.push_back(coeffs[i]);
  rgb_camera_info.height = 480;
  rgb_camera_info.width = 640;

  //if(requested_camera_serial_number == "612203002922")  // hack, only publish the passive near to the default topic
  _rgb_image_pub.publish(*rgb_img, rgb_camera_info);
}


// Return Unix (Epoch) time
std::string GetUnixTime() {
  std::time_t unit_time = std::time(nullptr);
  std::stringstream ss;
  ss << unit_time;
  return ss.str();
}

bool FileExists(const std::string &filename) {
  std::ifstream file(filename);
  return (!file.fail());
}

// trim from end
static inline std::string &rtrim(std::string &s) {
  s.erase(std::find_if(s.rbegin(), s.rend(),
                       std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
  return s;
}


void publish_pointcloud2_msg(std::string requested_camera_serial_number)
{
  for (int dy = 0; dy < frame_height; dy++) {
    for (int dx = 0; dx < frame_width; dx++) {
      int i = (frame_width * dy + dx);
      realsense_xyzrgb_cloud->points[i].x = cloud_buffer_pts[i*3+0];
      realsense_xyzrgb_cloud->points[i].y = cloud_buffer_pts[i*3+1];
      realsense_xyzrgb_cloud->points[i].z = cloud_buffer_pts[i*3+2];

      int r = cloud_buffer_rgb[i*3+0];
      int g = cloud_buffer_rgb[i*3+1];
      int b = cloud_buffer_rgb[i*3+2];
      realsense_xyzrgb_cloud->points[i].rgba = (0 << 24) | (r << 16) | (g << 8) | b;
    }
  }

  pcl::PCLPointCloud2 pcl_xyzrgb_pc2;
  pcl::toPCLPointCloud2 (*realsense_xyzrgb_cloud, pcl_xyzrgb_pc2);

  sensor_msgs::PointCloud2 realsense_xyzrgb_cloud2;
  pcl_conversions::moveFromPCL(pcl_xyzrgb_pc2, realsense_xyzrgb_cloud2);

  realsense_xyzrgb_cloud2.header.seq = header_sequence_id;
  realsense_xyzrgb_cloud2.header.stamp = header_time_stamp;
  realsense_xyzrgb_cloud2.header.frame_id = rgb_frame_id;  //registered to rgb

  if(requested_camera_serial_number == "612203002922")  // hack, only publish the passive near to the default topic
    pointcloud_pub.publish (realsense_xyzrgb_cloud2);

  realsense_xyzrgb_cloud2.header.frame_id = rgb_frame_id + "_" + requested_camera_serial_number;  //registered to rgb
  map_pointcloud_pub[rtrim(requested_camera_serial_number)].publish(realsense_xyzrgb_cloud2);
}

bool srv_set_save_data(std_srvs::SetBool::Request& req,
                       std_srvs::SetBool::Response& res){
  save_data = req.data;
}

bool srv_start_lasers(realsense_camera::start_lasers::Request& req,
                      realsense_camera::start_lasers::Response& res){

  try {
    if (req.bin_id == 0) { active_dev_bin0->set_option(rs::option::f200_laser_power, 16); }
    else if (req.bin_id == 1) { active_dev_bin1->set_option(rs::option::f200_laser_power, 16); }
    else if (req.bin_id == 2) { active_dev_bin2->set_option(rs::option::f200_laser_power, 16); }
    res.success = true;
  }
  catch (int e){
    res.success = false;
  }

  printf("Laser of bin_id %d camera is activated", req.bin_id);
}

bool srv_stop_lasers(realsense_camera::start_lasers::Request& req,
                     realsense_camera::start_lasers::Response& res){

  try {
    if (req.bin_id == 0) { active_dev_bin0->set_option(rs::option::f200_laser_power, 0); }
    else if (req.bin_id == 1) { active_dev_bin1->set_option(rs::option::f200_laser_power, 0); }
    else if (req.bin_id == 2) { active_dev_bin2->set_option(rs::option::f200_laser_power, 0); }
    res.success = true;
  }
  catch (int e){
    res.success = false;
  }

  printf("Laser of bin_id %d camera is deactivated", req.bin_id);
}

bool callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  return true;
}

// Service: stream data from sensor
bool srv_capture(realsense_camera::snapshot::Request  &req,
                 realsense_camera::snapshot::Response &res) {
  double totaltstart, tstart;
  totaltstart = tic();
  tstart = tic();

  // Check if requested device exists
  int device_idx = 0;
  std::string requested_camera_serial_number = req.camera_serial_number;
  ROS_INFO_STREAM_NAMED("realsense", std::string("Request data from from sensor ") + requested_camera_serial_number);
  while (requested_camera_serial_number.length() < 12)
    requested_camera_serial_number = requested_camera_serial_number + " ";
  if(map_ser_to_idx.find(rtrim(requested_camera_serial_number)) == map_ser_to_idx.end()){
    ROS_ERROR_STREAM_NAMED("realsense", "No device matches input serial number[" << requested_camera_serial_number << "]. Returning empty data.");
    return true;
  }
  device_idx = map_ser_to_idx[rtrim(requested_camera_serial_number)];

  print_toc(tstart, "Prepare");

  // Disable laser for previous camera if it is still active
  tstart = tic();
  if (active_dev->get_option(rs::option::f200_laser_power) != 0)
    active_dev->set_option(rs::option::f200_laser_power, 0);
  print_toc(tstart, "Disable laser");

  // Start laser for requested device
  tstart = tic();
  active_dev_idx = device_idx;  // a global variable
  active_dev = ctx.get_device(active_dev_idx);
  active_dev->set_option(rs::option::f200_laser_power, 16);
  print_toc(tstart, "Start laser");

  tstart = tic();
  for (int i = 0; i < 6; i++) active_dev->wait_for_frames(); // wait several frames for the laser on/off effect to take place
  print_toc(tstart, "Wait 6 frames");

  // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
  tstart = tic();
  const uint16_t * depth_data = (const uint16_t *)active_dev->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
  print_toc(tstart, "Get depth frame");

  // Retrieve color data, which was previously configured as a 640 x 480 x 3 image of 8-bit color values
  tstart = tic();
  const uint8_t * color_data = (const uint8_t *)active_dev->get_frame_data(rs::stream::rectified_color);
  print_toc(tstart, "Get color frame");

  // Create 3D point cloud
  tstart = tic();
  float depth_scale = active_dev->get_depth_scale();
  std::vector<double> xyzoffset = map_pointcloud_xyzoffset[rtrim(requested_camera_serial_number)];
  if(!correct_depth)  // if we don't want to correct depth (usually during camera calibration), set the offset to zero
    xyzoffset[2] = 0.0;

  rs::intrinsics color_cam_k = active_dev->get_stream_intrinsics(rs::stream::rectified_color);

  print_toc(tstart, "Create 3D point cloud prep");


  tstart = tic();
  for (int dy = 0; dy < frame_height; dy++) {
    int row_shift = dy * frame_width;
    for (int dx = 0; dx < frame_width; dx++) {
      int row_shift_plus_dx_times_3 = (row_shift + dx) * 3;

      // Retrieve the raw and aligned-to-color depth value and map it into a depth in meters
      uint16_t depth_value = depth_data[row_shift + dx];
      float depth_in_meters = depth_value * depth_scale;

      // Add RGB data to global frame buffer
      cloud_buffer_rgb[row_shift_plus_dx_times_3 + 0] = color_data[row_shift_plus_dx_times_3 + 0];
      cloud_buffer_rgb[row_shift_plus_dx_times_3 + 1] = color_data[row_shift_plus_dx_times_3 + 1];
      cloud_buffer_rgb[row_shift_plus_dx_times_3 + 2] = color_data[row_shift_plus_dx_times_3 + 2];

      // Skip over pixels with a depth value of zero, which is used to indicate no data
      if (depth_value == 0) {

        // Add empty point to global frame buffer
        cloud_buffer_pts[row_shift_plus_dx_times_3 + 0] = 0;
        cloud_buffer_pts[row_shift_plus_dx_times_3 + 1] = 0;
        cloud_buffer_pts[row_shift_plus_dx_times_3 + 2] = 0;
        continue;
      }

      // Map from pixel coordinates in the depth image to pixel coordinates in the color image
      rs::float2 depth_pixel = {(float)dx, (float)dy};
      rs::float3 depth_point = color_cam_k.deproject(depth_pixel, depth_in_meters);

      // Add XYZ point to global point cloud buffer
      cloud_buffer_pts[row_shift_plus_dx_times_3 + 0] = depth_point.x + depth_point.x / depth_point.z * xyzoffset[2];
      cloud_buffer_pts[row_shift_plus_dx_times_3 + 1] = depth_point.y + depth_point.y / depth_point.z * xyzoffset[2];
      cloud_buffer_pts[row_shift_plus_dx_times_3 + 2] = depth_point.z + xyzoffset[2];
    }
  }
  print_toc(tstart, "Create 3D point cloud copying");

  // Shutdown laser for requested device
  tstart = tic();
  active_dev->set_option(rs::option::f200_laser_power, 0);
  print_toc(tstart, "Shutdown laser for requested device");

  // Save RGB-D frame as image
  if (save_data) {

    // Set frame filename
    std::ostringstream frame_name;
    frame_name << std::setw(6) << std::setfill('0') << frame_idx[device_idx];
    frame_idx[device_idx]++;
    std::string unix_time = GetUnixTime();

    // Save aligned depth frame to disk
    float * depth_buffer = new float[frame_height * frame_width];
    for (size_t dy = 0; dy < frame_height; dy++)
      for (size_t dx = 0; dx < frame_width; dx++) {
        depth_buffer[dy * frame_width + dx] = cloud_buffer_pts[(dy * frame_width + dx) * 3 + 2];
      }
    std::string depth_file = data_path + rtrim(cam_serial_nums[device_idx]) + "/" + unix_time + ".depth.png";
    WriteDepth(depth_file, depth_buffer, frame_height, frame_width);

    // Save color frame to disk (RGB, 24-bit PNG)
    cv::Mat color_mat(frame_height, frame_width, CV_8UC3);
    for (int y = 0; y < frame_height; ++y){
      int row_shift = y * frame_width;
      for (int x = 0; x < frame_width; ++x) {
        int row_shift_plus_dx_times_3 = (row_shift + x) * 3;
        cv::Vec3b& bgr_value = color_mat.at<cv::Vec3b>(y, x);
        bgr_value[0] = color_data[row_shift_plus_dx_times_3 + 2]; // Blue
        bgr_value[1] = color_data[row_shift_plus_dx_times_3 + 1]; // Green
        bgr_value[2] = color_data[row_shift_plus_dx_times_3 + 0]; // Red
      }
    }
    std::vector<int> compression_params;
//    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);
    std::string color_file = data_path + rtrim(cam_serial_nums[device_idx]) + "/" + unix_time + ".color.png";
    imwrite(color_file, color_mat, compression_params);

    // Save file paths in ROS message
    res.file_paths.push_back(data_path + "cam.info.txt");
    res.file_paths.push_back(color_file);
    res.file_paths.push_back(depth_file);
  } else {
    res.file_paths.push_back("");
    res.file_paths.push_back("");
    res.file_paths.push_back("");
  }

  // Save point cloud buffers to ROS message

  tstart = tic();
  int sz = frame_width * frame_height * 3;
  res.point_cloud_xyz.insert(res.point_cloud_xyz.end(), &cloud_buffer_pts[0], &cloud_buffer_pts[sz]);
  res.point_cloud_rgb.insert(res.point_cloud_rgb.end(), &cloud_buffer_rgb[0], &cloud_buffer_rgb[sz]);

  // Save intrinsics and extrinsics of depth and color cameras
  res.color_camera_intrinsics.insert(res.color_camera_intrinsics.end(), &color_cam_intrin[device_idx][0], &color_cam_intrin[device_idx][9]);
  print_toc(tstart, "Copy to service response");


  // publishing to rostopic
  tstart = tic();
  memcpy(rgb_cvmat_buffer, color_data, frame_width * frame_height * 3);
//  publish_rgb_image_msg(rgb_cvmat, rtrim(cam_serial_nums[device_idx]));
  print_toc(tstart, "Publish to rostopic rgb");

  tstart = tic();
  publish_pointcloud2_msg(requested_camera_serial_number);
  print_toc(tstart, "Publish to rostopic pointcloud");

  ROS_INFO_STREAM_NAMED("realsense", std::string("Captured point cloud and RGB data from sensor ") + req.camera_serial_number);

  print_toc(totaltstart, "Total");
  std::cout << std::endl;
  return true;
}

std::vector<double> get_xyzoffset_from_file(std::string serialnum) {
  double input[3] = {0.0, 0.0, 0.0};
  std::string filename = camerainfo_directory + rtrim(serialnum) + ".xyz.offset.txt";
  FILE* fid = fopen(filename.c_str(), "r");
  if(!fid)
    return std::vector<double>(input, input+3);

  int res = fscanf(fid, "%lf%lf%lf", &input[0], &input[1], &input[2]);
  fclose(fid);
  return std::vector<double>(input, input+3);
}

int main(int argc, char **argv) {

  // Setup ROS
  ros::init(argc, argv, "realsense_camera");
  ros::NodeHandle n;
  ros::ServiceServer service_stream = n.advertiseService("realsense_camera/capture", srv_capture);
  ros::ServiceServer service_stream2 = n.advertiseService("realsense_camera/set_save_data", srv_set_save_data);
//  ros::ServiceServer service_stream3 = n.advertiseService("realsense_camera/set_save_data1", srv_set_save_data);
  ros::ServiceServer start_lasers = n.advertiseService("realsense_camera/start_lasers", srv_start_lasers);
  ros::ServiceServer stop_lasers = n.advertiseService("realsense_camera/stop_lasers", srv_stop_lasers);
//  ros::ServiceServer service = n.advertiseService("my_service", callback);
  // Get parameters from ROS call
  ros::NodeHandle priv_nh("~");
  priv_nh.param("display", display, false);
  priv_nh.param("save_data",save_data, false);
  priv_nh.param("correct_depth",correct_depth, true);

  // initialize buffer
  rgb_cvmat_buffer = new uint8_t[frame_width * frame_height * 3];
  rgb_cvmat = cv::Mat(frame_height, frame_width, CV_8UC3, rgb_cvmat_buffer);
  rgb_cvmat_buffer_bin0 = new uint8_t[frame_width * frame_height * 3];
  rgb_cvmat_bin0 = cv::Mat(frame_height, frame_width, CV_8UC3, rgb_cvmat_buffer_bin0);
  rgb_cvmat_buffer_bin1 = new uint8_t[frame_width * frame_height * 3];
  rgb_cvmat_bin1 = cv::Mat(frame_height, frame_width, CV_8UC3, rgb_cvmat_buffer_bin1);
  image_transport::ImageTransport image_transport_(n);
  rgb_image_pub = image_transport_.advertiseCamera("realsense", 1);
  rgb_image_pub_bin0 = image_transport_.advertiseCamera("rgb_bin0", 1);
  rgb_image_pub_bin1 = image_transport_.advertiseCamera("rgb_bin1", 1);
  rgb_image_pub_bin2 = image_transport_.advertiseCamera("rgb_bin2", 1);
  depth_image_pub_bin0 = image_transport_.advertiseCamera("depth_bin0", 1);
  depth_image_pub_bin1 = image_transport_.advertiseCamera("depth_bin1", 1);
  depth_image_pub_bin2 = image_transport_.advertiseCamera("depth_bin2", 1);

  pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>(topic_pointcloud, 1);

  ros::Publisher pub_depth_arr_bin0 = n.advertise<std_msgs::Float32MultiArray>("/arc_1/depth_arr_bin0", 1);
  ros::Publisher pub_depth_arr_bin1 = n.advertise<std_msgs::Float32MultiArray>("/arc_1/depth_arr_bin1", 1);
  ros::Publisher pub_depth_arr_bin2 = n.advertise<std_msgs::Float32MultiArray>("/arc_2/depth_arr_bin2", 1);

  // prepare buffer for pcl
  realsense_xyzrgb_cloud.reset(new pcl::PointCloud<PointType>());
  realsense_xyzrgb_cloud->width = frame_width;
  realsense_xyzrgb_cloud->height = frame_height;
  realsense_xyzrgb_cloud->is_dense = false;
  realsense_xyzrgb_cloud->points.resize(frame_width * frame_height);

  // Obtain a list of devices currently present on the system
  device_count = ctx.get_device_count();
  ROS_INFO_STREAM_NAMED("realsense", "========= " << device_count<<  " DEVICES=========");
  if (!device_count) {
    ROS_INFO_STREAM_NAMED("realsense", "No device detected. Is it plugged in?\n");
    //printf("No device detected. Is it plugged in?\n");
    return 0;
  }

  // Create lock folder in tmpdata
  std::string lock_path = "/home/mcube/arcdata/tmpdata/locks";
  if (!FileExists(lock_path))
    int res = system(("mkdir " + lock_path).c_str());

  usleep(microseconds*10);
  for (int i = 0; i < device_count; ++i) {
    // Show the device name and information
    rs::device * dev = ctx.get_device(i);
    std::string serial = dev->get_serial();
    //std::cout << "Device " << i << ": " << dev->get_name() << ":\n";
    ROS_INFO_STREAM_NAMED("realsense", "Device " << i << "  Serial number: " << serial << "  USB Port ID:" << dev->get_usb_port_id());
    cam_serial_nums.push_back(serial);
    //ROS_INFO_STREAM_NAMED("realsense", "  Firmware version: " << dev->get_firmware_version() );
    //try { ROS_INFO_STREAM_NAMED("realsense",  "  USB Port ID: " << dev->get_usb_port_id() ); } catch (...) {}
    if (dev->supports(rs::capabilities::adapter_board)) std::cout << " Adapter Board Firmware version: " << dev->get_info(rs::camera_info::adapter_board_firmware_version) << "\n";
    if (dev->supports(rs::capabilities::motion_events)) std::cout << " Motion Module Firmware version: " << dev->get_info(rs::camera_info::motion_module_firmware_version) << "\n";

    // Remove default auto white balancing
    dev->set_option(rs::option::color_white_balance, 2800);
    dev->set_option(rs::option::color_enable_auto_white_balance, 0); // default: 9

    // Shut down IR laser
    dev->set_option(rs::option::f200_laser_power, 0);

    // color_white_balance

    // Configure all streams to run at VGA resolution at 60 frames per second
    dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
    dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);

    dev->start();

    // Get camera intrinsics of color sensor
    rs::intrinsics curr_color_cam_intrin = dev->get_stream_intrinsics(rs::stream::rectified_color);
    std::vector<float> color_K;
    color_K.push_back(curr_color_cam_intrin.fx); color_K.push_back(0.0f); color_K.push_back(curr_color_cam_intrin.ppx);
    color_K.push_back(0.0f); color_K.push_back(curr_color_cam_intrin.fy); color_K.push_back(curr_color_cam_intrin.ppy);
    color_K.push_back(0.0f); color_K.push_back(0.0f); color_K.push_back(1.0f);
    color_cam_intrin.push_back(color_K);

    map_ser_to_color_cam_intrin[rtrim(serial)] = color_K;
    map_ser_to_idx[rtrim(serial)] = i;
    map_pointcloud_pub[rtrim(serial)] = n.advertise<sensor_msgs::PointCloud2>(topic_pointcloud + "_" + rtrim(serial), 1);
    map_pointcloud_xyzoffset[rtrim(serial)] = get_xyzoffset_from_file(rtrim(serial));
    usleep(microseconds*2);
  }

  // Set first device as active device
  active_dev = ctx.get_device(active_dev_idx);
//  active_dev_bin0 = ctx.get_device(active_dev_idx);
  // active_dev->set_option(rs::option::f200_laser_power, 16);
  // active_dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
  // active_dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
  // active_dev->start();
  //std::cout << "Switching stream to camera: " << cam_serial_nums[active_dev_idx] << std::endl;
  ROS_INFO_STREAM_NAMED("realsense", "Switching stream to camera: " << cam_serial_nums[active_dev_idx]);

  //std::cout << "\nReady.\n" << std::endl;
  ROS_INFO_STREAM_NAMED("realsense", "Ready.");



  // Open a GLFW window to display our output
//  if (display) {
//    glfwInit();
//    win = glfwCreateWindow(1280, 480, "Realsense RGB-D Stream", nullptr, nullptr);
//    glfwMakeContextCurrent(win);
//    ROS_INFO_STREAM_NAMED("realsense", "OpenGL Display enabled.");
//  }

  // Create a data folder (with a unix time name) to save frames
  data_path = data_directory + "cameradata/" + GetUnixTime() + "/";
  int res = system(("mkdir -p " + data_path).c_str());

  // Save camera intrinsics of color sensor
  if (save_data) {
    for (int device_idx = 0; device_idx < device_count; ++device_idx) {

      std::string cam_K_file = camerainfo_directory + rtrim(cam_serial_nums[device_idx]) + ".intrinsics.txt";
      FILE *fp = fopen(cam_K_file.c_str(), "w");

      // Save camera serial number
      // fprintf(fp, "%s\n", cam_serial_nums[device_idx].c_str());

      // Create directory for camera data
      int res = system(("mkdir -p " + data_path + cam_serial_nums[device_idx]).c_str());
      frame_idx.push_back(0);

      // Save color camera intrinsics
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", color_cam_intrin[device_idx][0], color_cam_intrin[device_idx][1],
              color_cam_intrin[device_idx][2]);
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n", color_cam_intrin[device_idx][3], color_cam_intrin[device_idx][4],
              color_cam_intrin[device_idx][5]);
      fprintf(fp, "%15.8e\t %15.8e\t %15.8e\t\n\n", color_cam_intrin[device_idx][6],
              color_cam_intrin[device_idx][7], color_cam_intrin[device_idx][8]);
      fclose(fp);
    }
  }
  
  std::string camera_bin0_serial = "612203002922";
  std::string camera_bin1_serial = "616205001219";
  std::string camera_bin2_serial = "612205002211";
    
  int camera_bin0_id =3;
  int camera_bin1_id =5;
  int camera_bin2_id =1;
  
  for (int i=0;i<device_count;i++){
    boost::erase_all(cam_serial_nums[i], " ");

    if ((cam_serial_nums[i])==camera_bin0_serial){
      camera_bin0_id = i;
    }
    if ((cam_serial_nums[i])==camera_bin1_serial){
      camera_bin1_id = i;
    }
    if ((cam_serial_nums[i])==camera_bin2_serial){
      camera_bin2_id = i;
    }
  }
  
  //~ std::cout<< "[ INFO] camera_bin0_id: "<<camera_bin0_id<<std::endl;
  //~ std::cout<< "[ INFO] camera_bin1_id: "<<camera_bin1_id<<std::endl;
  //~ std::cout<< "[ INFO] camera_bin2_id: "<<camera_bin2_id<<std::endl;

  int key_state = 0;
  int im_width = 640;
  int im_height = 480;
  int bin_id = 0;
  int machine_per_bin = 2;
  float * input0_depth_buffer = new float[im_height * im_width];
  float * input1_depth_buffer = new float[im_height * im_width];

  //deactive previous camera
//  active_dev->set_option(rs::option::f200_laser_power, 0);
  // Set first device as active device
//  active_dev = ctx.get_device(active_dev_idx);
//  active_dev_bin0 = ctx.get_device(camera_bin0_id);
//  active_dev_bin0->set_option(rs::option::f200_laser_power, 16);
//  active_dev_bin1 = ctx.get_device(camera_bin1_id);
//  active_dev_bin1->set_option(rs::option::f200_laser_power, 16);
  // active_dev->set_option(rs::option::f200_laser_power, 16);
  // active_dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
  // active_dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
  // active_dev->start();
  //std::cout << "Switching stream to camera: " << cam_serial_nums[active_dev_idx] << std::endl;
  // Spacebar key state

//  ROS_INFO_STREAM_NAMED("realsense", "Switching stream to camera: " << cam_serial_nums[active_dev_idx]);

  //std::cout << "\nReady.\n" << std::endl;
  ROS_INFO_STREAM_NAMED("realsense", "Ready.");

  // Spacebar key state
//  int key_state = 0;
  int counter = 0;

//  if(display){
    ros::Rate r(50); // 50 hz

  //deactivate camera
  active_dev->set_option(rs::option::f200_laser_power, 0);
//  active_dev = ctx.get_device(active_dev_idx);
//  active_dev->set_option(rs::option::f200_laser_power, 16);


//      active_dev_bin0->set_option(rs::option::f200_laser_power, 0);
  active_dev_bin0 = ctx.get_device(camera_bin0_id);
  active_dev_bin1 = ctx.get_device(camera_bin1_id);
  active_dev_bin0->set_option(rs::option::f200_laser_power, 0);
//  active_dev_bin2 = ctx.get_device(camera_bin2_id);
//  active_dev_bin0->set_option(rs::option::f200_laser_power, 16);
//  active_dev_bin1->set_option(rs::option::f200_laser_power, 16);


    while ((!display && ros::ok()) || (display && ros::ok())) {

      // Track spacebar key presses
//      if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS && key_state == 0)
//        key_state++;

      // If spacebar key is pressed, stream next camera
//      if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_RELEASE && key_state >= 1) {
//        key_state = 0;
//        if (active_dev_idx == device_count - 1)
//          active_dev_idx = 0;
//        else
//          active_dev_idx++;

        // Disable previous camera
//        active_dev->set_option(rs::option::f200_laser_power, 0);

        // Enable next camera
//        active_dev = ctx.get_device(active_dev_idx);
//        active_dev->set_option(rs::option::f200_laser_power, 16);
//        ROS_INFO_STREAM_NAMED("realsense", "Switching stream to camera: " << cam_serial_nums[active_dev_idx]);
//      }

      // Get new RGBD frame data
//      if (display) glfwPollEvents();
//      active_dev->wait_for_frames();
      active_dev_bin0->wait_for_frames();
      active_dev_bin1->wait_for_frames();
//      active_dev_bin2->wait_for_frames();

//      active_dev_bin0->set_option(rs::option::f200_laser_power, 16);
//      active_dev_bin1->set_option(rs::option::f200_laser_power, 16);


      //get rgb

      const uint8_t * color_data_bin0 = (const uint8_t *)active_dev_bin0->get_frame_data(rs::stream::rectified_color);
      const uint8_t * color_data_bin1 = (const uint8_t *)active_dev_bin1->get_frame_data(rs::stream::rectified_color);
//      const uint8_t * color_data_bin2 = (const uint8_t *)active_dev_bin2->get_frame_data(rs::stream::rectified_color);
      memcpy(rgb_cvmat_buffer_bin0, color_data_bin0, frame_width * frame_height * 3);
      memcpy(rgb_cvmat_buffer_bin1, color_data_bin1, frame_width * frame_height * 3);
//      memcpy(rgb_cvmat_buffer_bin2, color_data_bin2, frame_width * frame_height * 3);
      //publish rgb dat
      publish_rgb_image_msg(rgb_cvmat_bin0, rtrim(cam_serial_nums[camera_bin0_id]), rgb_image_pub_bin0);
      publish_rgb_image_msg(rgb_cvmat_bin1, rtrim(cam_serial_nums[camera_bin1_id]), rgb_image_pub_bin1);
//      publish_rgb_image_msg(rgb_cvmat_bin2, rtrim(cam_serial_nums[camera_bin2_id]), rgb_image_pub_bin2);
      //get depth
      const uint16_t * depth_data_bin0 = (const uint16_t *)active_dev_bin0->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
      const uint16_t * depth_data_bin1 = (const uint16_t *)active_dev_bin1->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
//      const uint16_t * depth_data_bin2 = (const uint16_t *)active_dev_bin2->get_frame_data(rs::stream::depth_aligned_to_rectified_color);
      float depth_scale_bin0 = active_dev_bin0->get_depth_scale();
      float depth_scale_bin1 = active_dev_bin1->get_depth_scale();
//      float depth_scale_bin2 = active_dev_bin2->get_depth_scale();

      publish_depth(depth_data_bin0, im_height, im_width, depth_image_pub_bin0, counter,  rtrim(cam_serial_nums[camera_bin0_id]));
      publish_depth(depth_data_bin1, im_height, im_width, depth_image_pub_bin1, counter,  rtrim(cam_serial_nums[camera_bin1_id]));

//        }

//      if (display) {
//        glClear(GL_COLOR_BUFFER_BIT);
//        glPixelZoom(1, -1);
//
//        // Display depth data by linearly mapping depth between 0 and 2 meters to the red channel
//        glRasterPos2f(-1, 1);
//        glPixelTransferf(GL_RED_SCALE, 0xFFFF * active_dev_bin0->get_depth_scale() / 2.0f);
//        glDrawPixels(640, 480, GL_RED, GL_UNSIGNED_SHORT, active_dev_bin0->get_frame_data(rs::stream::depth_aligned_to_rectified_color));
//        glPixelTransferf(GL_RED_SCALE, 1.0f);
//
//        // Display depth data by linearly mapping depth between 0 and 2 meters to the red channel
//        glRasterPos2f( 0, 1);
//        glPixelTransferf(GL_RED_SCALE, 0xFFFF * active_dev_bin1->get_depth_scale() / 2.0f);
//        glDrawPixels(640, 480, GL_RED, GL_UNSIGNED_SHORT, active_dev_bin1->get_frame_data(rs::stream::depth_aligned_to_rectified_color));
//        glPixelTransferf(GL_RED_SCALE, 1.0f);        // Display depth data by linearly mapping depth between 0 and 2 meters to the red channel



        // Display color image as RGB triples
//        glRasterPos2f(0, 1);
//        const uint8_t * color_data = (const uint8_t *)active_dev->get_frame_data(rs::stream::rectified_color);
//        glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, color_data);

//        memcpy(rgb_cvmat_buffer, color_data, frame_width * frame_height * 3);
//        publish_rgb_image_msg(rgb_cvmat, rtrim(cam_serial_nums[active_dev_idx]), rgb_image_pub);

//        glfwSwapBuffers(win);
//      }

      // ROS loop
      ros::spinOnce();
      r.sleep();
      counter ++;
    }
//  }
//  else{
//    ros::Rate r(60); // 1000 hz
//    while(ros::ok()){
//      const uint8_t * color_data = (const uint8_t *)active_dev->get_frame_data(rs::stream::rectified_color);
//      memcpy(rgb_cvmat_buffer, color_data, frame_width * frame_height * 3);
////      publish_rgb_image_msg(rgb_cvmat, rtrim(cam_serial_nums[active_dev_idx]));
//      ros::spinOnce();
//      r.sleep();
//    }
//  }

  return 0;
}
