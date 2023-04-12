#include <iostream>
#include <memory>
#include <thread>
#include "serial/serial.h"
#include "camera/include/camera.h"
#include "detector/include/armor_detect.h"
#include "Message.hpp"
#include "zmq.hpp"
#include "glog/logging.h"
#include "wit-motion/imu_receive.h"
#include "debug.h"
#include "Record.hpp"

std::shared_ptr<SerialPort> serial;
std::shared_ptr<Camera> camera;
std::shared_ptr<ArmorDetect> autoaim;
std::shared_ptr<WT> wit_motion;
extern Publisher<cv::Mat> display_pub_;
Subscriber<cv::Mat> display_sub_(&display_pub_);


int main(int argc, char* argv[]) {

    FLAGS_log_dir = "../glog";
    FLAGS_alsologtostderr = 1;
    FLAGS_max_log_size = 100;
    FLAGS_stop_logging_if_full_disk = true;
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Ares2023 Sentry Vision Start!!!";



#ifndef TEST

    serial = std::make_shared<SerialPort>("/dev/stm", 115200);
//    wit_motion = std::make_shared<WT>("/dev/imu",115200);
    camera = std::make_shared<Camera>("KE0200120159", 960, 768);
#endif
    autoaim = std::make_shared<ArmorDetect>();
#ifndef TEST
    std::thread serial_thread(&SerialPort::receive_thread, serial);
//    std::thread wit_thread(&WT::receive_thread, wit_motion);
    std::thread camera_thread(&Camera::camera_stream_thread, camera);
#endif
    std::thread autoaim_thread(&ArmorDetect::run, autoaim);

    void* context = zmq_ctx_new();
    void* publisher = zmq_socket(context,ZMQ_PUB);
    int bind = zmq_bind(publisher, "tcp://*:9000");
    Record recorder;
    int sub_status = -1;
#ifdef DEBUG
    while (1){

        cv::Mat src;
        src = display_sub_.subscribe(sub_status);
        if(sub_status ==0){
            continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
//        cv::resize(src,src,cv::Size(480,384));
//        std::cout<<"send"<<"\n";
        std::vector<uchar> buffer;
        int quality = 60; //0-100
        std::vector<int> compress_params;
        compress_params.push_back(IMWRITE_JPEG_QUALITY);
        compress_params.push_back(quality);
        cv::imencode(".jpg", src, buffer, compress_params);
        zmq_send(publisher,buffer.data(),buffer.size(), ZMQ_NOBLOCK);
//        cv::Mat video = cv::imdecode(buffer,CV_LOAD_IMAGE_COLOR);
//        recorder.writeVideo(video);
    }
#endif

    autoaim_thread.join();
#ifndef TEST
    camera_thread.join();
    serial_thread.join();
#endif
//    wit_thread.join();

}
