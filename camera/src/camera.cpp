//
// Created by quonone on 23-1-29.
//

#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include <fmt/color.h>
#include <string.h>
#include "../include/camera.h"
#include "glog/logging.h"

Publisher<Camdata> cam_publisher(2);

Camera::Camera(const char *SN, const int width, const int height) :
        image_width(width), image_height(height), cameraSN_(SN) {
    tic = std::make_shared<Tictok>();
    GX_STATUS status = Config();
    if (status != GX_STATUS_SUCCESS) {
//        fmt::print(fg(fmt::color::red), "Config Camera Faile...");
        LOG(ERROR) <<"Config Camera Failed...";
        exit(-1);
    }
    cam0_info.sn_str = cameraSN_;
    cam0_info.SN = &cam0_info.sn_str[0];

    cam0 = std::make_unique<MercureDriver>(cam0_info);
    cam0->InitCamera();
    if (cam0->status != GX_STATUS_SUCCESS) {
        LOG(ERROR) <<"Initial Camera Failed, Please check your camera ";
        exit(-1);
    }

    status = GXStreamOn(cam0->hDevice_);
    if (status != GX_STATUS_SUCCESS) {
//        fmt::print(fg(fmt::color::red), "Camera start stream error!!!");
        LOG(ERROR) <<"Camera start streaming error...";
        exit(-1);
    }
    origin_buff = new char[image_height * image_width * 3];
    camera_offline = 0;
}

Camera::~Camera() {
    delete[]origin_buff;
}

void Camera::camera_stream_thread() {
    while (true) {
        GX_STATUS status = GXDQBuf(cam0->hDevice_, &pFrameBuffer, 1000);
        if (status == GX_STATUS_SUCCESS) {
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS) {
                double time_stamp = tic->this_time();  //ms
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
                    time_stamp = time_stamp/1000;
                    Camdata cam_data = std::make_pair(time_stamp, raw_image);
                    cam_publisher.publish(cam_data);
                }
            }
            status = GXQBuf(cam0->hDevice_, pFrameBuffer);
        }else{
//            std::cout<<"[error ]can not get camera frame ..\n";
            LOG(ERROR) << "can not get camera frame ...";
            camera_offline++;
            if(camera_offline>5){
                LOG(FATAL) << "Camera offline more than 5 seconds, good bye !";
//                exit(-1);
            }
        }
    }
}