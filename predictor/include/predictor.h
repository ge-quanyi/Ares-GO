//
// Created by quonone on 23-2-22.
//

#ifndef ARES_CV_PREDICTOR_H
#define ARES_CV_PREDICTOR_H


#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include "../../include/data_type.h"
#include "kalmanFilter.hpp"
#include "anglesolver.hpp"

class Predictor {
public:
    explicit Predictor(const std::string& params_path);
    ~Predictor()=default;

    void reset();
    void predict(const Armor& armor,  cv::Point3f& cam_pred,const RobotInfo& robot);

private:

    float evaluate_threshold = 1000.0;
    double shoot_delay ;
    const std::string& file_path_;
    bool inited = false;
    std::vector<Armor> armor_seq;
    Armor current_armor;
    std::shared_ptr<ares::KalmanFilter> kf;
    std::shared_ptr<AngleSolver> anglesolver;
    RobotInfo robot_;

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



};


#endif //ARES_CV_PREDICTOR_H
