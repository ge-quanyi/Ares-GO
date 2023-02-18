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
    ovinfer = std::make_shared<OvInference>("../detector/model/rm-net16.xml");
}
void ArmorDetect::run() {
    cv::VideoCapture video("/home/quonone/Videos/video.mp4");

    while(true){

//        if(!camera->raw_image_pub.size()){
//            std::this_thread::sleep_for(std::chrono::milliseconds(1));
//            continue;
//        }
        cv::Mat src;
        video.read(src);
        if(src.empty())
            break;
//        double time_stamp;
//        {
//            std::lock_guard<std::mutex> lg2(sub_lock);
//            src = camera->raw_image_pub.front().second.clone();
//            time_stamp = camera->raw_image_pub.front().first;
//            camera->raw_image_pub.pop_front();
//        }
        if(src.empty())
            continue;
        ///TODO: armor detect and predict, then serial send
        std::vector<OvInference::Detection> results;
        ovinfer->infer(src, results);
        if(results.size() == 0)
            locked_id=-1;
//        std::cout<< "results size "<<results.size()<<std::endl;
        //红色id >= 9  && <=17
        int IF_WE_RED = 1;
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
        // 筛选完颜色
        OvInference::Detection final_obj;
        if(!locked_id){
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
                locked_id = final_obj.class_id;
            }
        }else{
            for (const auto re : results){
                if(re.class_id == locked_id){
                    final_obj = re;
                    break;
                }
            }
        }


        cv::putText(src,"lock_ed id "+ std::to_string(locked_id),cv::Point (15,30),
        cv::FONT_HERSHEY_COMPLEX_SMALL,1,cv::Scalar(255,0,0));
        cv::line(src, final_obj.obj.p1, final_obj.obj.p3, cv::Scalar(0, 0, 255), 3);
        cv::line(src, final_obj.obj.p2, final_obj.obj.p4, cv::Scalar(0, 0, 255), 3);
        cv::line(src, final_obj.obj.p1, final_obj.obj.p2, cv::Scalar(0, 0, 255), 3);
        cv::line(src, final_obj.obj.p3, final_obj.obj.p4, cv::Scalar(0, 0, 255), 3);
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
