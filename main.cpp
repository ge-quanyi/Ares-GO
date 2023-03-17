#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include "serial/serial.h"
#include "camera/include/camera.h"
#include "detector/include/armor_detect.h"
#include<opencv2/imgcodecs.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

std::shared_ptr<SerialPort> serial;
std::shared_ptr<Camera> camera;
std::shared_ptr<ArmorDetect> autoaim;
std::mutex main_sub;

int main() {

    serial = std::make_shared<SerialPort>("/dev/stm", 115200);
    camera = std::make_shared<Camera>("KE0200120159", 640, 480);
    autoaim = std::make_shared<ArmorDetect>();

    std::thread serial_thread(&SerialPort::receive_thread, serial);
    std::thread camera_thread(&Camera::camera_stream_thread, camera);
    std::thread autoaim_thread(&ArmorDetect::run, autoaim);

//    while(true){
//        cv::Mat src;
//        if(!autoaim->getdata(src)){
//            std::this_thread::sleep_for(std::chrono::milliseconds(3));
//            continue;
//        }
//        if(src.empty())
//            continue;
//
////        cv::imshow("video", src);
////        cv::waitKey(1);
//    }
    autoaim_thread.join();
    camera_thread.join();
    serial_thread.join();

}
