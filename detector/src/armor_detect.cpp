//
// Created by quonone on 23-1-31.
//
#include "../include/armor_detect.h"
#include "../../include/data_type.h"
#include <fmt/core.h>
#include <fmt/color.h>
#include <thread>
#include <mutex>
#include "../../wit-motion/imu_receive.h"
#include "debug.h"
#include "glog/logging.h"

Subscriber<Camdata> cam_subscriber(&cam_publisher);
//Subscriber<RobotInfo> serial_sub_(&serial_publisher);
Publisher<cv::Mat> display_pub_(1);
extern std::shared_ptr<SerialPort> serial;
extern std::shared_ptr<WT> wit_motion;

ArmorDetect::ArmorDetect() {
    tic = std::make_unique<Tictok>();
    pnpsolver = std::make_shared<PNPSolver>("../params/daheng159.yaml");
//    ovinfer = std::make_shared<OvInference>("../detector/model/rm-net16.xml");
    ovinfer = std::make_shared<Inference>("../detector/model/opt-0527-001.xml");
//    ovinfer = std::make_shared<Inference>("../detector/model/yolox16.xml");
//    ekfpredictor = std::make_shared<EKFPredictor>();
    kfpredictor = std::make_shared<Predictor>("../params/params.yaml");
    as = std::make_shared<AngleSolver>();
//    num_c = std::make_shared<Classifier>("../detector/model/fc.onnx", "../detector/model/label.txt", 0.7);
    tracking_roi = cv::Rect(0,0,0,0);
    image_empty_cnt = 0;
    std::cout<<"detector inited "<<std::endl;
}

void ArmorDetect::color_check(const char color, std::vector<ArmorObject> &results) {

    for (auto i = results.begin(); i != results.end();) {
        if (color == 'r') { //detect blue
            if (i->color == 1 || i->color == 2)
                i = results.erase(i);
            else
                ++i;
        } else {
            if (i->color == 0 || i->color ==2 )
                i = results.erase(i);
            else
                ++i;
        }
    }

}

void ArmorDetect::base_check(ArmorObject &final_obj) {
    double w = get_distance(final_obj.apex[0],final_obj.apex[3]);
    double h = get_distance(final_obj.apex[0],final_obj.apex[1]);
    double wh_rate = w/h;
    std::cout<<"wh rate cheack "<<wh_rate<<"\n";

    if(wh_rate > 3 ){
        if((final_obj.cls !=1 && final_obj.cls!=3 && final_obj.cls!=4) || (final_obj.prob < 0.95)){
            final_obj.cls = 0;
        }
    }else{
        if(((final_obj.cls != 3) && (final_obj.cls!=4)) || (final_obj.prob < 0.9)){
            final_obj.cls = 0;
        }
    }
}

void ArmorDetect::armor_sort(ArmorObject &final_obj, std::vector<ArmorObject> &results, cv::Mat &src) {
    if (results.size() == 0) {
        lose_cnt++;
        if (lose_cnt > 3) {
            lose_cnt = 0;
            locked_id = 0; //lose
        }
    } else
        lose_cnt = 0;

    if (results.size() == 1) {
        final_obj = results[0];
        locked_id = final_obj.cls;
    } else {
        if (locked_id == 0) { //already has not locked id yet
            double min_dis = 10000;
            int min_idx = -1;
            for (int i = 0; i < results.size(); ++i) {
                double obj_center_x =
                        0.25 * (results[i].pts[0].x + results[i].pts[1].x + results[i].pts[2].x + results[i].pts[3].x);
                double obj_center_y =
                        0.25 * (results[i].pts[0].y + results[i].pts[1].y + results[i].pts[2].y + results[i].pts[3].y);
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
                    locked_id = final_obj.cls;
                    lock_cnt = 0;
                }

            }
        } else {
            for (const auto &re: results) {
                if (re.cls == locked_id) {
                    final_obj = re;
                    break;
                }
            }
            lose_cnt++;
        }
    }

    //防止击打基地



    if(locked_id > 0){
        cv::Rect tmp = final_obj.rect;
        tmp.x -= 0.5*tmp.width;
        tmp.y -= tmp.height;
        tmp.height += 2*tmp.height;
        tmp.width += tmp.width;

        if(tmp.x<0){tmp.x=0;}
        if(tmp.y<0){tmp.y=0;}
        if(tmp.width+tmp.x>src.cols){tmp.width = src.cols-tmp.x;}
        if(tmp.height+tmp.y>src.rows){tmp.height = src.rows-tmp.y;}
        tracking_roi = tmp;
    }else{
        tracking_roi = cv::Rect (0,0,0,0);
    }


}

void ArmorDetect::draw_target(const ArmorObject &obj, cv::Mat &src) {
    cv::line(src, obj.apex[0], obj.apex[2], cv::Scalar(0, 255, 0), 2);
    cv::line(src, obj.apex[1], obj.apex[3], cv::Scalar(0, 255, 0), 2);
    cv::line(src, obj.apex[0], obj.apex[1], cv::Scalar(0, 255, 0), 2);
    cv::line(src, obj.apex[2], obj.apex[3], cv::Scalar(0, 255, 0), 2);
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


bool ArmorDetect::if_shoot(const cv::Point3f& cam_) {
    if(fabs(cam_.y )<0.1 && fabs(cam_.z)<0.15)
        return true;
    else
        return false;
}

#define SEND_IMG 1

void ArmorDetect::run() {
#ifdef TEST
    cv::VideoCapture cap(0);
#endif
    while (true) {

        try {
            cv::Mat src;
            double time_stamp;
#ifndef TEST
            int DATA_STATUS = -1;
            Camdata data = cam_subscriber.subscribe(DATA_STATUS); //传递value
            if(image_empty_cnt>40){
                LOG(FATAL)<<"CAN NOT GET IMAGE QUEUE ,exit";
            }
            if(!DATA_STATUS){
                double pitch, yaw, dis;
                pitch = yaw = dis = 0;
                char *send_data = new char[6];

                send_data[0] = int16_t(1000 * pitch);
                send_data[1] = int16_t(1000 * pitch) >> 8;
                send_data[2] = int16_t(1000 * yaw);
                send_data[3] = int16_t(1000 * yaw) >> 8;
                send_data[4] = int16_t(100 * dis);
                send_data[5] = int16_t(100 * dis) >> 8;

                bool status = serial->SendBuff('0x30', send_data, 6);
                delete[] send_data;
                image_empty_cnt++;
                LOG(INFO)<<"data queue is empty!! ,lose cnt " + to_string(image_empty_cnt)<<"\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            image_empty_cnt = 0;
            src = data.second; //获取头即可
            time_stamp = data.first;
#endif
#ifdef TEST
            cap.read(src);
#endif
            if(src.empty())
                continue;
//            RobotInfo robot_  = serial_sub_.subscribe();

            if(tracking_roi.width>0){
                auto t0 = std::chrono::system_clock::now();
                cv::rectangle(src,tracking_roi,cv::Scalar(255,0,0),2);
                cv::Mat roi = src(tracking_roi).clone();
                cv::Mat dst;
                int add_top = tracking_roi.y;
                int add_bottom = src.rows-(tracking_roi.y+tracking_roi.height);
                int add_left = tracking_roi.x;
                int add_right = src.cols - (tracking_roi.x+tracking_roi.width);
//                cv::Mat dst(src.rows, src.cols,CV_8UC3,cv::Scalar(100,100,100));
//                roi.copyTo(dst(tracking_roi));
                cv::copyMakeBorder(roi,dst,add_top,add_bottom,add_left,add_right,cv::BORDER_CONSTANT,cv::Scalar(100,100,100));
//                copyMakeBorder(roi, dst,0,top,0,right,cv::BORDER_CONSTANT,cv::Scalar(100,100,100));
//                std::cout<<"dst size "<<dst.rows<<" "<<dst.cols<<"\n";
                src = dst;
                auto t1 = std::chrono::system_clock::now();
                double ppp = std::chrono::duration<double, milli>(t1-t0).count();
//                std::cout<<"pp"<<ppp<<"\n";

            }
#ifdef TEST
            RobotInfo robot_ = {'r',0.999,0,0,0,22};
#else
            RobotInfo robot_ = serial->robot_;
#endif
//            WitInfo wit_ = wit_motion->wt;
//            robot_.q[0] = wit_.q[0];
//            robot_.q[1] = wit_.q[1];
//            robot_.q[2] = wit_.q[2];
//            robot_.q[3] = wit_.q[3];
            as->getEuler(robot_);
//            std::cout<<"wit q"<<" "<<wit_.q[0]<<" "<<wit_.q[1]<<" "<<wit_.q[2]<<" "<<wit_.q[3]<<"\n";
//            std::cout<<"euler "<<" "<<as->euler[0]<<" "<<as->euler[1]<<" "<<as->euler[2]<<"\n";
            if(robot_.bullet_speed == 0){robot_.bullet_speed=27;}
            if(robot_.bullet_speed < 15){robot_.bullet_speed = 15;}
//            as->getEuler(robot_);
//            std::cout<<"robot speed "<<robot_.bullet_speed<<"\r\n";
//            std::cout<<"euler "<<as->euler[0]<< " "<<as->euler[1]<<" "<<as->euler[2]<<"\r\n";
//            std::cout<<"q "<<robot_.q[0]<<" "<<robot_.q[1]<<" "<<robot_.q[2]<<" "<<robot_.q[3]<<"\r\n";
//            std::vector<OvInference::Detection> results;
            std::vector<ArmorObject> results;
            // choose tradition or ai
            ovinfer->detect(src, results);
            color_check(robot_.color, results);
//            cv::Mat dst;
//            detect(src,dst,1,results);
            
//            OvInference::Detection final_obj;
            ArmorObject final_obj;
            final_obj.cls = 0;           //check if armor
            armor_sort(final_obj, results, src);
            if(final_obj.cls>0){ base_check(final_obj);}
            if(final_obj.cls==0){
                tracking_roi.width = 0;
            }
            double pitch, yaw, dis;
            char cmd = 0;
            pitch = yaw = dis = 0;
            if (final_obj.cls > 0) {
                //draw
                draw_target(final_obj, src);
                cv::Point2f center = cv::Point2f ((final_obj.apex[0].x+final_obj.apex[1].x+final_obj.apex[2].x+final_obj.apex[3].x)/4,\
                (final_obj.apex[0].y+final_obj.apex[1].y+final_obj.apex[2].y+final_obj.apex[3].y)/4);

                auto cam_ = pnpsolver->get_cam_point(final_obj);

                cv::circle(src,center,5,cv::Scalar(0,255,0),-1);

                Armor armor;
                armor.time_stamp = time_stamp;
                armor.cam_point_ = cam_;
                armor.id = final_obj.cls;
                cv::Point3f cam_pred;
#ifdef EKF
                ekfpredictor->predict(armor, cam_pred, robot_);
#else
                kfpredictor->predict(armor, cam_pred, robot_);
#endif
                as->getAngle(cam_pred, pitch, yaw, dis, robot_);
//                as->getAngle_nofix(cam_pred,pitch,yaw,dis);
                cv::Point2f center_pred = pnpsolver->cam2pixel(cam_pred);
                cv::circle(src,center_pred,10,cv::Scalar(0,0,255),2);
//                std::cout<<"cam "<<cam_.y <<"  "<< cam_.z<<"\r\n";

                if(if_shoot(cam_))
                    cmd = 1;
                else
                    cmd = 0;

                if(dis>10){final_obj.cls==0;}
            }
//
            if (final_obj.cls > 0) {
                pitch_last = pitch;
                yaw_last = yaw;
                dis_last = dis;
                id_last = final_obj.cls;
            } else {
                if (id_last > 0) {
                    final_obj.cls = id_last;
                    pitch = pitch_last;
                    yaw = yaw_last;
                    dis = dis_last;
                    pitch_last = dis_last = yaw_last = 0;
                    id_last = 0;
                } else {
                    pitch = yaw = dis = 0;

                }

            }
            char *send_data = new char[6];

            send_data[0] = int16_t(1000 * pitch);
            send_data[1] = int16_t(1000 * pitch) >> 8;
            send_data[2] = int16_t(1000 * yaw);
            send_data[3] = int16_t(1000 * yaw) >> 8;
            send_data[4] = int16_t(100 * dis);
            send_data[5] = int16_t(100 * dis) >> 8;
#ifndef TEST
            bool status = serial->SendBuff(cmd, send_data, 6);
#endif
            delete[] send_data;
            tic->fps_calculate(autoaim_fps);
            if(final_obj.cls>0){
                cv::putText(src, "id " + std::to_string(locked_id)+" color: "+std::to_string(final_obj.color) + "conf "+
                                                                                                                std::to_string(final_obj.prob), cv::Point(final_obj.apex[0].x-5, final_obj.apex[0].y-5),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255));
            }

            cv::putText(src, "fps " + std::to_string(autoaim_fps), cv::Point(15, 30),
                        cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 0, 0));
            fmt::print(fg(fmt::color::green), "object data :pitch {:.3f},yaw {:.3f}, dis {:.3f}, FPS {}. \r\n", pitch, yaw, dis, autoaim_fps);


#ifdef DEBUG
            img_send_cnt++;
            if(img_send_cnt>3){
                display_pub_.publish(src);
                img_send_cnt = 0;
            }
#endif
        } catch (...) {
            LOG(ERROR)<<" AUTOAIM thread has certain errors. ";
            std::cout << "[WARNING] camera not ready." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }

    }
}
