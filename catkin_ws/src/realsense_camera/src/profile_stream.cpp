#include "stream.h"

//void CallCapture()
//{
//    passive_vision::state::Request &req;
//    passive_vision::state::Response &res;
//
//    int bin_id = 0;
//     Define debug saving path
//    std::string debug_data_path = data_directory + "loopdata/passive_vision/" + GetUnixTime() + "/";
//    res.file_paths.push_back(debug_data_path);
//     Process dirty command
//    if (std::strcmp(req.update_command.substr(0,5).c_str(),"dirty") == 0) {
//        std::cout << "[MASTER]: Recieved dirty call: " << req.update_command.c_str() << std::endl;
//        if (std::strcmp(req.update_command.substr(6,1).c_str(),"1") == 0)
//            bin_is_dirty_override[0] = true;
//        else
//            bin_is_dirty_override[0] = false;
//        if (std::strcmp(req.update_command.substr(7,1).c_str(),"1") == 0)
//            bin_is_dirty_override[1] = true;
//        else
//            bin_is_dirty_override[1] = false;
//        if (std::strcmp(req.update_command.substr(8,1).c_str(),"1") == 0)
//            bin_is_dirty_override[2] = true;
//        else
//            bin_is_dirty_override[2] = false;
//        if (std::strcmp(req.update_command.substr(9,1).c_str(),"1") == 0)
//            bin_is_dirty_override[3] = true;
//        else
//            bin_is_dirty_override[3] = false;
//        return true;
//
//         Add state command to queue
//    } else if (std::strcmp(req.update_command.c_str(),"") != 0) {
//        command_queues[bin_id].push(req.update_command);
//        std::cout << "[MASTER]: Recieved state command: " << req.update_command << std::endl;
//
//         Save state command to text file
//        std::ofstream outFile;
//        outFile.open(data_directory + "tmpdata/state-command.txt");
//        outFile << std::string(req.update_command + "\n");
//        outFile.close();
//        save_debug_paths[bin_id] = debug_data_path;
//        save_debug_requested[bin_id] = true;
//        save_debug_timestamps[bin_id] = grasp_timestamps[bin_id];
//        return true;
//    }
//
//     Return camera to world transformations
//    for (int i = 0; i <= 2; i+=2)
//        for (int r = 0; r < 4; ++r)
//            for (int c = 0; c < 4; ++c)
//                res.camera_extrinsics.push_back(camera_pose[bin_id][i](r,c));
//
//    {
//        std::lock_guard<std::mutex> scoped_lock(service_mutex[bin_id]);
//
//        res.suction_points = suction_predictions[bin_id];
//        res.suction_object_list = suction_object_list[bin_id];
//        res.suction_object_confidence = suction_object_confidence[bin_id];
//        res.grasp_proposals = grasp_predictions[bin_id];
//        res.grasp_object_list = grasp_object_list[bin_id];
//        res.grasp_object_confidence = grasp_object_confidence[bin_id];
//        res.flush_grasp_proposals = flush_grasp_predictions[bin_id];
//        res.flush_grasp_object_list = flush_grasp_object_list[bin_id];
//        res.flush_grasp_object_confidence = flush_grasp_object_confidence[bin_id];
//        res.height_map = height_maps[bin_id];
//        res.state_object_list_ontop = state_object_list_ontop[bin_id];
//        res.state_object_pose = state_object_pose[bin_id];
//        res.state_object_confidence = state_object_confidence[bin_id];
//        res.state_object_visibility = state_object_visibility[bin_id];
//        res.camera_timestamp = camera_timestamps[bin_id];
//        res.heightmap_timestamp = heightmap_timestamps[bin_id];
//        res.state_timestamp = state_timestamps[bin_id];
//        res.suction_timestamp = suction_timestamps[bin_id];
//        res.grasp_timestamp = grasp_timestamps[bin_id];
//    }
//
//     Request copy debug images
//    if (std::strcmp(save_debug_timestamps[bin_id].c_str(),grasp_timestamps[bin_id].c_str()) != 0) {
//        save_debug_paths[bin_id] = debug_data_path;
//        save_debug_requested[bin_id] = true;
//        save_debug_timestamps[bin_id] = grasp_timestamps[bin_id];
//    }
//
//    return true;
//}
//
int main(int argc, char **argv) {

    // Setup ROS
//    ros::init(argc, argv, "realsense_camera");
//    ros::NodeHandle n;
//    ros::ServiceServer service_stream = n.advertiseService("realsense_camera/capture", srv_capture);
//
//     initialize buffer
//    rgb_cvmat_buffer = new uint8_t[frame_width * frame_height * 3];
//    rgb_cvmat = cv::Mat(frame_height, frame_width, CV_8UC3, rgb_cvmat_buffer);
//    image_transport::ImageTransport image_transport_(n);
//    rgb_image_pub = image_transport_.advertiseCamera("realsense", 1);
//    pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>(topic_pointcloud, 1);

    // prepare buffer for pcl
//    realsense_xyzrgb_cloud.reset(new pcl::PointCloud<PointType>());
//    realsense_xyzrgb_cloud->width = frame_width;
//    realsense_xyzrgb_cloud->height = frame_height;
//    realsense_xyzrgb_cloud->is_dense = false;
//    realsense_xyzrgb_cloud->points.resize(frame_width * frame_height);

    // Obtain a list of devices currently present on the system
//    device_count = ctx.get_device_count();
//    ROS_INFO_STREAM_NAMED("realsense", "========= " << device_count << " DEVICES=========");
//    if (!device_count) {
//        ROS_INFO_STREAM_NAMED("realsense", "No device detected. Is it plugged in?\n");
        //printf("No device detected. Is it plugged in?\n");
//        return 0;
//    }

//    usleep(microseconds);
//    for (int i = 0; i < device_count; ++i) {
//         Show the device name and information
//        rs::device *dev = ctx.get_device(i);
//        std::string serial = dev->get_serial();
//        cam_serial_nums.push_back(serial);
        // Remove default auto white balancing
//        dev->set_option(rs::option::color_white_balance, 2800);
//        dev->set_option(rs::option::color_enable_auto_white_balance, 0); // default: 9
//        dev->set_option(rs::option::f200_laser_power, 16);
        // Configure all streams to run at VGA resolution at 60 frames per second
//        dev->enable_stream(rs::stream::depth, 640, 480, rs::format::z16, 60);
//        dev->enable_stream(rs::stream::color, 640, 480, rs::format::rgb8, 60);
//        dev->start();
        // Get camera intrinsics of color sensor
//        rs::intrinsics curr_color_cam_intrin = dev->get_stream_intrinsics(rs::stream::rectified_color);
//        std::vector<float> color_K;
//        color_K.push_back(curr_color_cam_intrin.fx);
//        color_K.push_back(0.0f);
//        color_K.push_back(curr_color_cam_intrin.ppx);
//        color_K.push_back(0.0f);
//        color_K.push_back(curr_color_cam_intrin.fy);
//        color_K.push_back(curr_color_cam_intrin.ppy);
//        color_K.push_back(0.0f);
//        color_K.push_back(0.0f);
//        color_K.push_back(1.0f);
//        color_cam_intrin.push_back(color_K);
//        map_ser_to_color_cam_intrin[rtrim(serial)] = color_K;
//        map_ser_to_idx[rtrim(serial)] = i;
//        map_pointcloud_pub[rtrim(serial)] = n.advertise<sensor_msgs::PointCloud2>(
//        topic_pointcloud + "_" + rtrim(serial), 1);
//        map_pointcloud_xyzoffset[rtrim(serial)] = get_xyzoffset_from_file(rtrim(serial));
//        usleep(microseconds);
//    }
//
//    for (int i = 0; i < 100; ++i){
//        std::cout << "Iteration: " << i << std::endl;
//        CallCapture();
//    }
//
    return 0;
}