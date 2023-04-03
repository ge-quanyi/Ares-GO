#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include "serial/serial.h"
#include "camera/include/camera.h"
#include "detector/include/armor_detect.h"
#include "Message.hpp"
#include "zmq.hpp"
#include "glog/logging.h"
#include "wit-motion/imu_receive.h"

std::shared_ptr<SerialPort> serial;
std::shared_ptr<Camera> camera;
std::shared_ptr<ArmorDetect> autoaim;
std::shared_ptr<WT> wit_motion;
extern Publisher<cv::Mat> display_pub_;
Subscriber<cv::Mat> display_sub_(&display_pub_);

int main(int argc, char* argv[]) {

    FLAGS_log_dir = "../glog";
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Ares2023 Computer Vision Start!!!";

    serial = std::make_shared<SerialPort>("/dev/stm", 115200);
    wit_motion = std::make_shared<WT>("/dev/ttyUSB0",115200);
    camera = std::make_shared<Camera>("KE0200120159", 640, 480);
    autoaim = std::make_shared<ArmorDetect>();

    std::thread serial_thread(&SerialPort::receive_thread, serial);
    std::thread wit_thread(&WT::receive_thread, wit_motion);
    std::thread camera_thread(&Camera::camera_stream_thread, camera);
    std::thread autoaim_thread(&ArmorDetect::run, autoaim);

    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context,ZMQ_PUB);
    int bind = zmq_bind(publisher, "tcp://*:9000");


    while (1){

        cv::Mat src;
        src = display_sub_.subscribe();
//        std::cout<<"send"<<"\n";
        std::vector<uchar> buffer;
        int quality = 60; //0-100
        std::vector<int> compress_params;
        compress_params.push_back(IMWRITE_JPEG_QUALITY);
        compress_params.push_back(quality);
        cv::imencode(".jpg", src, buffer, compress_params);
        zmq_send(publisher,buffer.data(),buffer.size(), ZMQ_NOBLOCK);
    }

    autoaim_thread.join();
    camera_thread.join();
    serial_thread.join();
    wit_thread.join();

}
