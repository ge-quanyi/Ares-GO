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
#include "Message.hpp"

typedef std::pair<double, cv::Mat> Camdata;
class Camera{
public:
    explicit Camera(const char* SN, const int width, const int height);
    ~Camera();
    void camera_stream_thread();

private:
    const char *cameraSN_;
    const int image_width, image_height;
    camera_config cam0_info;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    char *origin_buff ;
    std::unique_ptr<MercureDriver> cam0;
    PGX_FRAME_BUFFER pFrameBuffer;
    std::shared_ptr<Tictok> tic;
    int camera_offline;
};
#endif //ARES_CV_CAMERA_H
