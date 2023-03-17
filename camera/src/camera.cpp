//
// Created by quonone on 23-1-29.
//

#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include <fmt/color.h>
#include <mutex>
#include <string.h>
#include "../include/camera.h"



Camera::Camera(const char *SN, const int width, const int height) :
        image_width(width), image_height(height), cameraSN_(SN) {
    tic = std::make_shared<Tictok>();
    GX_STATUS status = Config();
    if (status != GX_STATUS_SUCCESS) {
        fmt::print(fg(fmt::color::red), "Config Camera Faile...");
        exit(-1);
    }
    cam0_info.sn_str = cameraSN_;
    cam0_info.SN = &cam0_info.sn_str[0];

    cam0 = std::make_unique<MercureDriver>(cam0_info);
    cam0->InitCamera();
    if (cam0->status != GX_STATUS_SUCCESS) {
        fmt::print(fg(fmt::color::red), "Initial Camera Faile...");
        exit(-1);
    }

    status = GXStreamOn(cam0->hDevice_);
    if (status != GX_STATUS_SUCCESS) {
        fmt::print(fg(fmt::color::red), "Camera start stream error!!!");
        exit(-1);
    }
    origin_buff = new char[image_height * image_width * 3];
}

Camera::~Camera() {
    delete[]origin_buff;
}


void Camera::putdata(const Camdata& data) {
    std::lock_guard<std::mutex> cam_lg(cam_lock);
    if(raw_image_pub.size()>5){
        raw_image_pub.pop();
    }
    raw_image_pub.push(data);

}

bool Camera::get_cam_data(double &time_point, cv::Mat &img) {
    std::lock_guard<std::mutex> cam_lg(cam_lock);
    if(!raw_image_pub.size())
        return false;
    img = raw_image_pub.front().second.clone();
    time_point = raw_image_pub.front().first;
    raw_image_pub.pop();
    return true;
}

void Camera::camera_stream_thread() {
    while (true) {
        GX_STATUS status = GXDQBuf(cam0->hDevice_, &pFrameBuffer, 1000);
        if (status == GX_STATUS_SUCCESS) {
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS) {
                double time_stamp = tic->this_time();
                memset(origin_buff,0,pFrameBuffer->nWidth* pFrameBuffer->nHeight* 3 * sizeof(char));
                DX_BAYER_CONVERT_TYPE cvtype = RAW2RGB_NEIGHBOUR;           //选择插值算法
                DX_PIXEL_COLOR_FILTER nBayerType = BAYERBG;              //选择图像Bayer格式
                VxInt32 DxStatus = DxRaw8toRGB24(const_cast<void *>(pFrameBuffer->pImgBuf), origin_buff,
                                                 pFrameBuffer->nWidth,
                                                 pFrameBuffer->nHeight, cvtype, nBayerType, false);
                if (DxStatus == DX_OK) {
                    //copy image data
//                    tic->fps_calculate();
                    cv::Mat raw_image(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC3);
                    memcpy(raw_image.data, origin_buff, pFrameBuffer->nWidth*pFrameBuffer->nHeight*3);
                    Camdata cam_data = std::make_pair(time_stamp, raw_image);
                    putdata(cam_data);
//                    fmt::print("buff size is {}\n", raw_image_pub.size());
                }
            }
            status = GXQBuf(cam0->hDevice_, pFrameBuffer);
        }
    }
}