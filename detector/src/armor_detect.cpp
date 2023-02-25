//
// Created by quonone on 23-1-31.
//
#include "../include/armor_detect.h"
#include "../../include/data_type.h"

#include <thread>
#include <mutex>

std::mutex sub_lock;
std::mutex pub_lock;

ArmorDetect::ArmorDetect() {
    tic = std::make_unique<Tictok>();
    pnpsolver = std::make_shared<PNPSolver>("../params/daheng159.yaml");
    ovinfer = std::make_shared<OvInference>("../detector/model/rm-net16.xml");
    anglesolver = std::make_shared<AngleSolver>();

}

void ArmorDetect::color_check(const char color, std::vector<OvInference::Detection> &results) {
    int IF_WE_RED = -1;
    if(color == 'r')
        IF_WE_RED = 1;
    for (auto i = results.begin(); i != results.end();) {
        if(IF_WE_RED){
            if(i->class_id<9)
                i = results.erase(i);
            else
                ++i;
        }else {
            if(i->class_id>9)
                i = results.erase(i);
            else
                ++i;
        }
    }

}

void ArmorDetect::armor_sort(OvInference::Detection &final_obj, std::vector <OvInference::Detection> &results, cv::Mat& src) {
    if(results.size() == 0){
        lose_cnt++;
        if(lose_cnt>3){
            lose_cnt = 0;
            locked_id=-1; //lose
        }
    }else
        lose_cnt = 0;

    if(results.size() == 1){
        final_obj = results[0];
        locked_id = final_obj.class_id;
    }
    else{
        if(locked_id == -1){ //already has not locked id yet
            double min_dis = 10000;
            int min_idx = -1;
            for (int i = 0; i < results.size(); ++i) {
                double obj_center_x = 0.25*(results[i].obj.p1.x+results[i].obj.p2.x+results[i].obj.p3.x+results[i].obj.p4.x);
                double obj_center_y = 0.25*(results[i].obj.p1.y+results[i].obj.p2.y+results[i].obj.p3.y+results[i].obj.p4.y);
                double dis = fabs(obj_center_x - src.cols/2)+ fabs(obj_center_y-src.rows/2);
                if(dis < min_dis){
                    min_dis = dis;
                    min_idx = i;
                }
            }
            if(min_idx>=0){
                final_obj = results[min_idx];
                lock_cnt++;
                if(lock_cnt>3){
                    locked_id = final_obj.class_id;
                    lock_cnt = 0;
                }

            }
        }else{
            for (const auto re : results){
                if(re.class_id == locked_id){
                    final_obj = re;
                    break;
                }
            }
            lose_cnt++;
        }
    }
}

void ArmorDetect::draw_target(const OvInference::Detection &obj, cv::Mat &src) {
    cv::line(src, obj.obj.p1, obj.obj.p3, cv::Scalar(0, 0, 255), 3);
    cv::line(src, obj.obj.p2, obj.obj.p4, cv::Scalar(0, 0, 255), 3);
    cv::line(src, obj.obj.p1, obj.obj.p2, cv::Scalar(0, 0, 255), 3);
    cv::line(src, obj.obj.p3, obj.obj.p4, cv::Scalar(0, 0, 255), 3);
}


void ArmorDetect::run() {
//    cv::VideoCapture video("/home/ares/Videos/video.mp4");

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
        std::vector<OvInference::Detection> results;
        ovinfer->infer(src, results);
        RobotInfo robot_ = serial->robotInfo_;
        color_check(robot_.color, results);

        OvInference::Detection final_obj;
        final_obj.class_id = -1;
        armor_sort(final_obj, results, src);
        cv::putText(src,"lock_ed id "+ std::to_string(locked_id),cv::Point (15,30),
        cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(255,0,0));

        draw_target(final_obj,src);

        auto cam_ = pnpsolver->get_cam_point(final_obj);
        double pitch, yaw, dis;
        anglesolver->getAngle(cam_,pitch,yaw,dis, robot_);
        auto w_point = anglesolver->cam2abs(cam_,robot_);
        ///TODO: predict

        auto pred_cam_ = anglesolver->abs2cam(w_point, robot_);

        char* send_data = new char[6];
        char cmd = 0x31;
        send_data[0] = int16_t (1000*pitch);
        send_data[1] = int16_t (1000*pitch) >> 8;
        send_data[2] = int16_t (1000*yaw);
        send_data[3] = int16_t (1000*yaw) >> 8;
        send_data[4] = int16_t (1000*dis);
        send_data[5] = int16_t (1000*dis) >> 8;
        serial->SendBuff(cmd, send_data, 6);
        delete[] send_data;

//        std::cout<<"cam_ "<<cam_<<std::endl;

        ///TODO: transport image to display
        {
            std::lock_guard<std::mutex> lg3(pub_lock);
            image_to_display_.push_back(src);
            if(image_to_display_.size()>2)
                image_to_display_.pop_front();
        }

        tic->fps_calculate();
    }

}
