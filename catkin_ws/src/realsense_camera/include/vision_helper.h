//
// Created by mcube on 6/28/17.
//

#ifndef PROJECT_VISION_HELPER_H
#define PROJECT_VISION_HELPER_H

#include <string>
#include <sys/file.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#include "realsense_camera/snapshot.h"
#include <mutex>
#include "ros/ros.h"

std::mutex realsense_mutex;

// trim from end
std::string &rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(),
                         std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
    return s;
}

bool FileExists(const std::string &filename) {
    std::ifstream file(filename);
    return (!file.fail());
}

int lock_camera(){
    int fd = open("/home/mcube/arcdata/tmpdata/locks/arc_vision.lock", O_RDWR | O_CREAT, 0666);
    int rc = flock(fd, LOCK_EX);
    return fd;
}

void unlock_camera(int fd){
    close(fd);
}

std::vector<float> LoadMatrixFromFile(std::string filename, int M, int N) {
    std::vector<float> matrix;
    FILE *fp = fopen(filename.c_str(), "r");
    for (int i = 0; i < M * N; i++) {
        float tmp;
        int iret = fscanf(fp, "%f", &tmp);
        matrix.push_back(tmp);
    }
    fclose(fp);
    return matrix;
}

void WriteDepth(const std::string &depth_file, float * depth_values, int im_height, int im_width) {
    cv::Mat depth_mat(im_height, im_width, CV_16UC1);
    for (size_t y = 0; y < im_height; y++)
        for (size_t x = 0; x < im_width; x++) {
            unsigned short depth_short = (unsigned short)(depth_values[y * im_width + x] * 10000);
            // depth_short = (depth_short >> 13 | depth_short << 3);
            depth_mat.at<unsigned short>(y, x) = depth_short;
        }
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//    compression_params.push_back(CV_IMWRITE_PXM_BINARY);
    compression_params.push_back(0);
    cv::imwrite(depth_file, depth_mat, compression_params);
}


void GetSnapshot(realsense_camera::snapshot & camera1, realsense_camera::snapshot & camera2, int bin_id, std::map<std::string, ros::ServiceClient> & realsense_client, std::vector<std::string> & machine_per_bin)
{
    int fd = lock_camera();

    // Set shutdown command to both machines
    // realsense_camera::snapshot realsense_srv_shutdown;
    // realsense_srv_shutdown.request.camera_serial_number = "";
    // realsense_client["arc_1"].call(realsense_srv_shutdown);
    // realsense_client["arc_2"].call(realsense_srv_shutdown);

    // {
    //     std::lock_guard<std::mutex> scoped_lock(realsense_mutex);

    // for (auto it = realsense_client.begin(); it != realsense_client.end(); it++) {
    //     if (it->first != machine_per_bin[bin_id])
    //         it->second.call(realsense_srv_shutdown);
    // }

    realsense_client[machine_per_bin[bin_id]].call(camera1);
    realsense_client[machine_per_bin[bin_id]].call(camera2);

    // }
    unlock_camera(fd);
}

#include <iomanip>
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

double toc(double t) {
  double s = tic();
  return s-t;
}

void print_toc(double t, std::string title) {
  double s = tic();
  std::cout << "[" << title << "] Elapsed time: " << std::setprecision(5) << s-t << "sec" << std::endl;
}


#endif //PROJECT_VISION_HELPER_H
