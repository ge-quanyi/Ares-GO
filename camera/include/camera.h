//
// Created by quonone on 23-1-31.
//

#ifndef ARES_CV_CAMERA_H
#define ARES_CV_CAMERA_H

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "Tictoc.hpp"
#include "../include/camera_api.h"


typedef std::pair<double, cv::Mat> Camdata;
class Camera{
public:
    explicit Camera(const char* SN, const int width, const int height);
    ~Camera();
    bool get_cam_data(double& time_point, cv::Mat& img);
    void camera_stream_thread();

private:
    const char *cameraSN_;
    const int image_width, image_height;
    int fps, fps_count;
    bool if_time_set = false;
    camera_config cam0_info;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    char *origin_buff ;
    std::unique_ptr<MercureDriver> cam0;
    PGX_FRAME_BUFFER pFrameBuffer;
    std::shared_ptr<Tictok> tic;
    // image queue
    std::queue<Camdata> raw_image_pub;

    std::mutex cam_lock;

    void putdata(const Camdata& data);
};
#endif //ARES_CV_CAMERA_H
