//
// Created by quonone on 23-1-31.
//
#include "../include/armor_detect.h"
#include <thread>
#include <mutex>

std::mutex sub_lock;
std::mutex pub_lock;
ArmorDetect::ArmorDetect() {
    tic = std::make_unique<Tictok>();
}
void ArmorDetect::run() {

    while(true){

        if(!camera->raw_image_pub.size()){
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        cv::Mat src;
        double time_stamp;
        {
            std::lock_guard<std::mutex> lg2(sub_lock);
            src = camera->raw_image_pub.front().second.clone();
            time_stamp = camera->raw_image_pub.front().first;
            camera->raw_image_pub.pop_front();
        }
        if(src.empty())
            continue;
        ///TODO: armor detect and predict, then serial send
        cv::putText(src,"test font ", cv::Point2d(30,30),\
        cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255,0,0),1);

        ///TODO: transport image to display
        {
            std::lock_guard<std::mutex> lg3(pub_lock);
            image_to_display_.push_back(src);
            if(image_to_display_.size()>5)
                image_to_display_.pop_front();
        }

        tic->fps_calculate();
    }

}
