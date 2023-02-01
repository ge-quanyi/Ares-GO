#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <crow.h>
#include "serial/serial.h"
#include "camera/include/camera.h"
#include "detector/include/armor_detect.h"

std::shared_ptr<SerialPort> serial;
std::shared_ptr<Camera> camera;
std::mutex main_sub;
int main() {
//    serial = std::make_shared<SerialPort>("/dev/stm", 115200);
    camera = std::make_shared<Camera>("KE0200120159", 960, 768);
    std::shared_ptr<ArmorDetect> autoaim = std::make_shared<ArmorDetect>();
//    std::thread serial_thread(&SerialPort::receive_thread, serial);
    std::thread camera_thread(&Camera::camera_stream_thread, camera);
    std::thread autoaim_thread(&ArmorDetect::run, autoaim);

    crow::SimpleApp app;
    CROW_ROUTE(app, "/")([](){
        return "Hello world";
    });
    app.port(18080).run();

//    while(true){
//        if(!autoaim->image_to_display_.size()){
//            std::this_thread::sleep_for(std::chrono::milliseconds(1));
//            continue;
//        }
//        cv::Mat src;
//        {
//            std::lock_guard<std::mutex> lg5(main_sub);
//            src = autoaim->image_to_display_.front();
//            autoaim->image_to_display_.pop_front();
//        }
//
//        if(src.empty())
//            continue;
//
//        cv::imshow("video", src);
//        cv::waitKey(1);
//    }

}
