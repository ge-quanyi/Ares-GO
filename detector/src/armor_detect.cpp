//
// Created by quonone on 23-1-31.
//
#include "../include/armor_detect.h"
#include "../../include/data_type.h"
#include <fmt/core.h>
#include <fmt/color.h>
#include <thread>
#include <mutex>


ArmorDetect::ArmorDetect() {
    tic = std::make_unique<Tictok>();
    pnpsolver = std::make_shared<PNPSolver>("../params/daheng159.yaml");
    ovinfer = std::make_shared<OvInference>("../detector/model/rm-net16.xml");
    predictor = std::make_shared<EKFPredictor>();
    as = std::make_shared<AngleSolver>();
}

void ArmorDetect::color_check(const char color, std::vector<OvInference::Detection> &results) {
    int IF_WE_RED = -1;
    if (color == 'r')
        IF_WE_RED = 1;
    for (auto i = results.begin(); i != results.end();) {
        if (IF_WE_RED) {
            if (i->class_id < 9)
                i = results.erase(i);
            else
                ++i;
        } else {
            if (i->class_id > 9)
                i = results.erase(i);
            else
                ++i;
        }
    }

}

void
ArmorDetect::armor_sort(OvInference::Detection &final_obj, std::vector<OvInference::Detection> &results, cv::Mat &src) {
    if (results.size() == 0) {
        lose_cnt++;
        if (lose_cnt > 3) {
            lose_cnt = 0;
            locked_id = -1; //lose
        }
    } else
        lose_cnt = 0;

    if (results.size() == 1) {
        final_obj = results[0];
        locked_id = final_obj.class_id;
    } else {
        if (locked_id == -1) { //already has not locked id yet
            double min_dis = 10000;
            int min_idx = -1;
            for (int i = 0; i < results.size(); ++i) {
                double obj_center_x =
                        0.25 * (results[i].obj.p1.x + results[i].obj.p2.x + results[i].obj.p3.x + results[i].obj.p4.x);
                double obj_center_y =
                        0.25 * (results[i].obj.p1.y + results[i].obj.p2.y + results[i].obj.p3.y + results[i].obj.p4.y);
                double dis = fabs(obj_center_x - src.cols / 2) + fabs(obj_center_y - src.rows / 2);
                if (dis < min_dis) {
                    min_dis = dis;
                    min_idx = i;
                }
            }
            if (min_idx >= 0) {
                final_obj = results[min_idx];
                lock_cnt++;
                if (lock_cnt > 3) {
                    locked_id = final_obj.class_id;
                    lock_cnt = 0;
                }

            }
        } else {
            for (const auto re: results) {
                if (re.class_id == locked_id) {
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

void ArmorDetect::putdata(const cv::Mat &img) {
    std::lock_guard<std::mutex> lg(pub_lock);
    image_to_display_.push(img);
    if (image_to_display_.size() > 5)
        image_to_display_.pop();
}

bool ArmorDetect::getdata(cv::Mat &src) {
    if (image_to_display_.empty())
        return false;
    src = image_to_display_.front().clone();
    image_to_display_.pop();
}

void ArmorDetect::run() {
//    cv::VideoCapture video("/home/ares/Videos/video.mp4");

    while (true) {

        try {
            cv::Mat src; //注意
            double time_stamp;
            if (!camera->get_cam_data(time_stamp, src)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
//            std::cout << "src " << src << std::endl;
            if(src.empty())
                continue;
            ///TODO: armor detect and predict, then serial send
            std::vector<OvInference::Detection> results;
            ovinfer->infer(src, results);
            RobotInfo robot_ = {'r',0,0,0,15};
//            serial->get_robot_data(robot_);

            fmt::print(fg(fmt::color::yellow), "robot data pitch {:.3f},yaw {:.3f}, roll {:.3f}, sp {:.3f}. \r\n",
                       robot_.ptz_pitch,robot_.ptz_yaw, robot_.ptz_roll, robot_.bullet_speed);
            color_check(robot_.color, results);

            OvInference::Detection final_obj;
            final_obj.class_id = -1;           //check if armor
            armor_sort(final_obj, results, src);
            cv::putText(src, "lock_ed id " + std::to_string(locked_id), cv::Point(15, 30),
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 0, 0));

            draw_target(final_obj, src);

            double pitch, yaw, dis;
            pitch = yaw = dis = 0;
            if (final_obj.class_id > -1) {
                auto cam_ = pnpsolver->get_cam_point(final_obj);
                Armor armor;
                armor.time_stamp = time_stamp;
                armor.cam_point_ = cam_;
                armor.id = final_obj.class_id;
                cv::Point3f cam_pred;
                predictor->predict(armor, cam_pred, robot_);
                as->getAngle_nofix(cam_pred, pitch, yaw, dis);
            }

            if (final_obj.class_id > -1) {
                pitch_last = pitch;
                yaw_last = yaw;
                dis_last = dis;
                id_last = final_obj.class_id;
            } else {
                if (id_last > -1) {
                    final_obj.class_id = id_last;
                    pitch = pitch_last;
                    yaw = yaw_last;
                    dis = dis_last;
                    pitch_last = dis_last = yaw_last = 0;
                    id_last = -1;
                } else {
                    pitch = yaw = dis = 0;

                }

            }
            char *send_data = new char[6];
            char cmd = 0x31;
            if (final_obj.class_id == -1) { cmd = 0x30; }
            send_data[0] = int16_t(1000 * pitch);
            send_data[1] = int16_t(1000 * pitch) >> 8;
            send_data[2] = int16_t(1000 * yaw);
            send_data[3] = int16_t(0000 * yaw) >> 8;
            send_data[4] = int16_t(100 * dis);
            send_data[5] = int16_t(100 * dis) >> 8;
            bool status = serial->SendBuff(cmd, send_data, 6);
            delete[] send_data;
            std::cout<<"id "<<final_obj.class_id<<std::endl;
//            fmt::print(fg(fmt::color::green), "object data {:.3f},{:.3f},{:.3f}. \r\n", pitch, yaw, dis);

//        std::cout<<"cam_ "<<cam_<<std::endl;
//        putdata(src);

            tic->fps_calculate();
        } catch (...) {
            std::cout << "[WARNING] camera not ready." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }
}
