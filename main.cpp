#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include "serial/serial.h"
#include "camera/include/camera.h"
#include "detector/include/armor_detect.h"
#include "Message.hpp"


std::shared_ptr<SerialPort> serial;
std::shared_ptr<Camera> camera;
std::shared_ptr<ArmorDetect> autoaim;
extern Publisher<cv::Mat> display_pub_;
Subscriber<cv::Mat> display_sub_(&display_pub_);

int main() {

    serial = std::make_shared<SerialPort>("/dev/stm", 115200);
    camera = std::make_shared<Camera>("KE0200120159", 640, 480);
    autoaim = std::make_shared<ArmorDetect>();

    std::thread serial_thread(&SerialPort::receive_thread, serial);
    std::thread camera_thread(&Camera::camera_stream_thread, camera);
    std::thread autoaim_thread(&ArmorDetect::run, autoaim);


    while(true){
        cv::Mat src;
        src = display_sub_.subscribe();
        if(src.empty())
            continue;
//        cv::imshow("video", src);
//        cv::waitKey(1);
    }
    autoaim_thread.join();
    camera_thread.join();
    serial_thread.join();

}
