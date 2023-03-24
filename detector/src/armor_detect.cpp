//
// Created by quonone on 23-1-31.
//
#include "../include/armor_detect.h"
#include "../../include/data_type.h"
#include <fmt/core.h>
#include <fmt/color.h>
#include <thread>
#include <mutex>


Subscriber<Camdata> cam_subscriber(&cam_publisher);
Subscriber<RobotInfo> serial_sub_(&serial_publisher);
Publisher<cv::Mat> display_pub_(1);
extern std::shared_ptr<SerialPort> serial;
ArmorDetect::ArmorDetect() {
    tic = std::make_unique<Tictok>();
    pnpsolver = std::make_shared<PNPSolver>("../params/ost.yaml");
    ovinfer = std::make_shared<OvInference>("../detector/model/rm-net16.xml");
    predictor = std::make_shared<EKFPredictor>();
    as = std::make_shared<AngleSolver>();
//    num_c = std::make_shared<Classifier>("../detector/model/fc.onnx", "../detector/model/label.txt", 0.7);
}

void ArmorDetect::color_check(const char color, std::vector<OvInference::Detection> &results) {

    for (auto i = results.begin(); i != results.end();) {
        if (color == 'r') {
            if (i->class_id < 9)
                i = results.erase(i);
            else
                ++i;
        } else {
//            std::cout<<"passsss red"<<std::endl;
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


//traditional
void ArmorDetect::detect(cv::Mat &src, cv::Mat &dst, const int team ,\
                        std::vector<OvInference::Detection>& objs) {
    cv::Mat gray_img, threshold_img;
    cv::cvtColor(src,gray_img,cv::COLOR_BGR2GRAY);
    cv::threshold(gray_img,threshold_img,200,255,cv::THRESH_BINARY);

    dst = threshold_img.clone();

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> white_hierarchy;
    findContours(dst, contours, white_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::cout<<"contours size "<<contours.size()<<std::endl;
    int index = -1;
    cv::RotatedRect lightBar_miniArea;
    LightBar light;
    std::vector<LightBar> lightbar_seq;
    for(const auto &contour: contours){
        index++;
        if(contour.size()<20)
            continue;
        lightBar_miniArea = minAreaRect(contour);
        //
        std::vector<cv::Point2f> points(4);//注意，不同opencv版本定义的点的顺序不一样
        lightBar_miniArea.points(points.data());
        if(lightBar_miniArea.size.width > lightBar_miniArea.size.height){
            light.angle = fabs(lightBar_miniArea.angle) + 90;
            light.height = lightBar_miniArea.size.width;
            light.width = lightBar_miniArea.size.height;
            light.p1 = (points[2] + points[3])/2;
            light.p2 = (points[0] + points[1])/2;
            ///todo sort point
            //light.points()
        }else{
            light.angle = fabs(lightBar_miniArea.angle);
            light.height = lightBar_miniArea.size.height;
            light.width = lightBar_miniArea.size.width;
            light.p1 = (points[1] + points[2])/2;
            light.p2 = (points[0] + points[3])/2;
//            std::cout<<"pu "<<light.point_up<<std::endl;
        }

        if((light.height / light.width) < 3)
            continue;
        if(light.angle<45 || light.angle>135){
            //check color
            int r_sum = 0, b_sum = 0;
            auto rect = lightBar_miniArea.boundingRect();
            if(rect.x>=0 && rect.y>=0 && rect.x+rect.width<= src.cols&& \
                rect.y>=0 && rect.height>=0 && rect.y+rect.height<=src.rows){
                auto roi_img = src(rect);
                for(int m = 0; m < roi_img.rows; ++m){
                    for (int n = 0; n < roi_img.cols; ++n) {
                        if(cv::pointPolygonTest(contour,cv::Point2f(rect.x+n, rect.y+m), false)>=0){
                            r_sum += roi_img.at<cv::Vec3b>(n,m)[2];
                            b_sum += roi_img.at<cv::Vec3b>(n,m)[0];
                        }
                    }
                }
            }
            if(team == 1){ //find red
                if(r_sum > b_sum){
                    cv::drawContours(src,contours,index,cv::Scalar(0,0,0),1);
//                                    cv::circle(src, light.p1,5, cv::Scalar(0,255,0),-1);
                    lightbar_seq.push_back(light);
                }
            }else if(team ==2){ //find blue
                if(r_sum < b_sum){
                    cv::drawContours(src,contours,index,cv::Scalar(0,0,0),1);
//                                    cv::circle(src, light.p1,5, cv::Scalar(0,255,0),-1);
                    lightbar_seq.push_back(light);
                }
            }
        }

    }

    std::cout<<"light bar size "<<lightbar_seq.size()<<std::endl;
    std::vector<Armor_t> armor_seq;
    Armor_t tmp_armor;
    OvInference::Detection obj;
    //match armor
    if(lightbar_seq.size() > 0){
        for(auto i = lightbar_seq.begin();i != lightbar_seq.end() - 1; i++){
            for(auto j = i+1; j != lightbar_seq.end(); j++){
                cv::Point2f vec_1 = cv::Point2f (i->p2.x - i->p1.x, i->p2.y - i->p1.y);
                cv::Point2f center_1 = cv::Point2f ((i->p2.x + i->p1.x)/2, (i->p2.y + i->p1.y)/2);
                cv::Point2f vec_2 = cv::Point2f (j->p2.x - j->p1.x, j->p2.y - j->p1.y);
                cv::Point2f center_2 = cv::Point2f ((j->p2.x + j->p1.x)/2, (j->p2.y + j->p1.y)/2);
                double vec1_length = sqrt(pow(vec_1.x,2)+pow(vec_1.y,2));
                double vec2_length = sqrt(pow(vec_2.x,2)+pow(vec_2.y,2));
                double vec_1_angle = i->angle;
                double vec_2_angle = j->angle;

                if(fabs(vec_1_angle - vec_2_angle) > 6)
                    continue;
                float max_l = std::max(vec1_length, vec2_length);
                float min_l = std::min(vec2_length, vec2_length);
                if(max_l / min_l > 1.5)
                    continue;

                //todo add rectangle or pingxing
                double light_dis = sqrt(pow(center_1.x-center_2.x,2) + pow(center_1.y-center_2.y,2));
                double rate_max = light_dis / min_l;
                double rate_min = light_dis / max_l;
                if(rate_max < 5 && rate_min>1){
                    cv::Point2f armor_center = cv::Point2f ((center_1.x+center_2.x)/2, (center_1.y+center_2.y)/2);

                    //todo add classier
                    float height = (vec2_length+vec2_length)/2;
                    float width = light_dis;
                    float armor_angle = (vec_2_angle+vec_1_angle)/2;
                    cv::RotatedRect armor_rect(armor_center,cv::Size(width,height), armor_angle);
                    cv::Rect armor_roi = armor_rect.boundingRect();
                    armor_roi.x += 0.1*armor_roi.width;
                    armor_roi.y -= 0.5*armor_roi.height;
                    armor_roi.height += armor_roi.height;
                    armor_roi.width -= 0.2*armor_roi.width;
                    if(armor_roi.x>=0 && armor_roi.y>=0&& armor_roi.x+armor_roi.width<=src.cols&&armor_roi.y+armor_roi.height<=src.rows){
                        cv::Mat roi_img = src(armor_roi);
                        int num = num_c->predict(roi_img);
                        std::cout<<"id "<<num<<std::endl;
                        tmp_armor = {i->p2, i->p1, j->p1, j->p2, armor_center,num};
                        //todo check if is armor
                        obj.obj = {armor_roi,tmp_armor.p1,tmp_armor.p2,tmp_armor.p3,tmp_armor.p4};
                        obj.class_id = num;
                        obj.confidence = 0.5;
                        if(num>0){
                            armor_seq.push_back(tmp_armor);
                            objs.push_back(obj);
                        }

                    }
                    //draw
                    cv::circle(src, tmp_armor.p1,5, cv::Scalar(0,255,0),2);
                    cv::circle(src, tmp_armor.p2,4, cv::Scalar(0,255,0),2);
                    cv::circle(src, tmp_armor.p3,3, cv::Scalar(0,255,0),2);
                    cv::circle(src, tmp_armor.p4,2, cv::Scalar(0,255,0),2);
                    cv::circle(src, armor_center,3, cv::Scalar(0,255,0),-1);
                    cv::line(src,tmp_armor.p1, tmp_armor.p3,cv::Scalar(0,0,255),2);
                    cv::line(src,tmp_armor.p2, tmp_armor.p4,cv::Scalar(0,0,255),2);
                    cv::line(src,tmp_armor.p1, tmp_armor.p2,cv::Scalar(0,0,255),2);
                    cv::line(src,tmp_armor.p3, tmp_armor.p4,cv::Scalar(0,0,255),2);

                }
            }
        }
        std::cout<<"armor size "<<armor_seq.size()<<std::endl;
        //

    }


}

void ArmorDetect::run() {

    while (true) {

        try {
            cv::Mat src;
            double time_stamp;
            Camdata data = cam_subscriber.subscribe();

            src = data.second.clone();
            time_stamp = data.first;
            if(src.empty())
                continue;

            RobotInfo robot_  = serial_sub_.subscribe();
            std::vector<OvInference::Detection> results;
            // choose tradition or ai
            ovinfer->infer(src, results);
            color_check(robot_.color, results);
//            cv::Mat dst;
//            detect(src,dst,1,results);
            
            OvInference::Detection final_obj;
            final_obj.class_id = -1;           //check if armor
            armor_sort(final_obj, results, src);

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
                as->getAngle_nofix(cam_, pitch, yaw, dis);
            }
//
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
            send_data[3] = int16_t(1000 * yaw) >> 8;
            send_data[4] = int16_t(100 * dis);
            send_data[5] = int16_t(100 * dis) >> 8;
            bool status = serial->SendBuff(cmd, send_data, 6);
            delete[] send_data;
            tic->fps_calculate(autoaim_fps);
            cv::putText(src, "id " + std::to_string(locked_id), cv::Point(15, 15),
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 0, 0));
            cv::putText(src, "fps " + std::to_string(autoaim_fps), cv::Point(15, 30),
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 0, 0));
            fmt::print(fg(fmt::color::green), "object data :pitch {:.3f},yaw {:.3f}, dis {:.3f}. \r\n", pitch, yaw, dis);
            display_pub_.publish(src);
        } catch (...) {
            std::cout << "[WARNING] camera not ready." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }
}
