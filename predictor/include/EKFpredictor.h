//
// Created by quonone on 23-3-17.
//

#ifndef ARES_CV_EKFPREDICTOR_H
#define ARES_CV_EKFPREDICTOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include "anglesolver.hpp"
#include "../../include/data_type.h"
#include "AdaptiveEKF.hpp"
#include <memory>

struct Predict {
    /*
     * 此处定义匀速直线运动模型  x1 = x0+v*delta_t
     * x1 = [x,v,]
     */
    template<class T>
    void operator()(const T x0[5], T x1[5]) {
        x1[0] = x0[0] + delta_t * x0[1];  //0.1
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1
        x1[3] = x0[3];  //100
        x1[4] = x0[4];  //0.01
    }
    double delta_t;
};

//  xyz
// pitch  y / (x2 + z2)
//yaw  x/z
template<class T>
void xyz2pyd(T xyz[3], T pyd[3]) {
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[1], ceres::sqrt(xyz[0] * xyz[0] + xyz[2] * xyz[2]));  // pitch
    pyd[1] = ceres::atan2(xyz[0], xyz[2]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);  // distance
}

struct Measure {
    /*
     * 工具函数的类封装
     */
    template<class T>
    void operator()(const T x[5], T y[3]) {
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};

class EKFPredictor{
private:
    double pitch_last, yaw_last, dis_last;
    int id_last;

    double last_time;
    double last_dis = 0;

    std::shared_ptr<AdaptiveEKF<5, 3>> ekf;


    RobotInfo robot_;
    float evaluate_threshold = 1000.0;
    bool inited = false;
    std::vector<Armor> armor_seq;
    Armor current_armor;
    std::unique_ptr<AngleSolver> anglesolver;
    void read_params();

    inline void abs2motion(cv::Point3f &abs_p_,cv::Point3f& motion_p_){
        motion_p_.x = abs_p_.y;
        motion_p_.y = abs_p_.z;
        motion_p_.z = -abs_p_.x;
    }

    inline void motion2abs(cv::Point3f &motion_p_, cv::Point3f& abs_p_){
        abs_p_.x = -motion_p_.z;
        abs_p_.y = motion_p_.x;
        abs_p_.z = motion_p_.y;
    }


public:

    explicit EKFPredictor();
    ~EKFPredictor()=default;

    void reset();
    void predict(const Armor& armor,  cv::Point3f& cam_pred,const RobotInfo& robot);
};
#endif //ARES_CV_EKFPREDICTOR_H
