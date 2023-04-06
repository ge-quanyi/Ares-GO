//
// Created by quonone on 23-2-22.
//

#include "../include/predictor.h"

Predictor::Predictor(const std::string &params_path):file_path_(params_path) {
    cv::FileStorage fs(file_path_, cv::FileStorage::READ);
    anglesolver = std::make_shared<AngleSolver>();
    if(!fs.isOpened()){
        std::cout<<"[ERROR ]predictor open params file error, please check \r\n";
        exit(-1);
    }
    Eigen::MatrixXd A(6,6);
    Eigen::MatrixXd P0(6,6);
    Eigen::MatrixXd H(3,6);
    Eigen::MatrixXd Q(6,6);
    Eigen::MatrixXd R(3,3);
    Eigen::MatrixXd I(6,6);
    A<< 1,0,0,0.5,0,0,
        0,1,0,0,0.5,0,
        0,0,1,0,0,0.5,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;
    P0.setIdentity(6,6);
    I.setIdentity(6,6);
    H << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;
    Q << fs["sentry_up"]["q00"], 0,0,0,0,0,
         0,fs["sentry_up"]["q11"],0,0,0,0,
         0,0,fs["sentry_up"]["q22"],0,0,0,
         0,0,0,fs["sentry_up"]["q33"],0,0,
         0,0,0,0,fs["sentry_up"]["q44"],0,
         0,0,0,0,0,fs["sentry_up"]["q55"];
    R << fs["sentry_up"]["r00"],0,0,
         0,fs["sentry_up"]["r11"],0,
         0,0,fs["sentry_up"]["r22"];
    shoot_delay = fs["sentry_up"]["shoot_delay"];

    kf = std::make_shared<ares::KalmanFilter>(A,P0,H,Q,R);
    fs.release();
    std::cout<<"q" << Q<<"\n shoot delay"<<shoot_delay<<"\n";
}

void Predictor::reset() {

    Eigen::VectorXd x0(6) ;
    x0 << current_armor.world_point_.x, current_armor.world_point_.y, current_armor.world_point_.z,0,0,0;
    kf->init(x0, current_armor.time_stamp);
    armor_seq.clear();
    std::cout<<"reset !!\n";
}

//
void Predictor::predict(const Armor& armor,  cv::Point3f& cam_pred,const RobotInfo& robot) {

    robot_ = robot;
    auto abs_point = anglesolver->cam2abs(armor.cam_point_, robot_);
    abs2motion(abs_point, current_armor.world_point_);
//    std::cout<<"abs "<<current_armor.world_point_<<"\r\n";

    current_armor.time_stamp = armor.time_stamp;
    current_armor.id = armor.id;
    current_armor.distance = sqrt(current_armor.world_point_.x*current_armor.world_point_.x + current_armor.world_point_.y*current_armor.world_point_.y + \
                                current_armor.world_point_.z*current_armor.world_point_.z);
    //predict
    if(!inited){
        reset();
        inited = true;
    }

    Eigen::VectorXd y0(3);
    y0 << current_armor.world_point_.x, current_armor.world_point_.y, current_armor.world_point_.z;

    kf->predict(y0,current_armor.time_stamp);
//    std::cout<<"x hat new"<<kf->X_hat_new<<"\n";
    //
    Eigen::VectorXd Ek(3); //3x1

    Ek = y0 - kf->H*kf->X_hat_new;
    Eigen::MatrixXd Dk(3,3);
    Dk=kf->H*kf->P.inverse()*kf->H.transpose()+kf->R; //3x3
    auto rk = Ek.transpose()*Dk.inverse()*Ek;
    std::cout<<"rk "<<rk<<"\n";

    if(rk > 0.005){
        inited = false;
    }

    ///TODO : result evalue
    kf->update(y0);
    double fly_time = current_armor.distance/robot_.bullet_speed + shoot_delay;
//    std::cout<<"fly time "<<fly_time<<"\n";
    std::cout<<"bulled speed "<<robot_.bullet_speed<<"\n";
    cv::Point3f result = cv::Point3f (kf->X_hat_new(0)+kf->X_hat_new(3)*fly_time,\
                                        kf->X_hat_new(1)+kf->X_hat_new(4)*fly_time,\
                                        kf->X_hat_new(2)+kf->X_hat_new(5)*fly_time);

    cv::Point3f abs_pred;
    //get final abs
    motion2abs(result, abs_pred);
//    std::cout<<"abs pred "<<abs_pred<<"\n";
    if(inited)
        cam_pred = anglesolver->abs2cam(abs_pred,robot_);
    else
        cam_pred = current_armor.cam_point_;

    if(armor_seq.size()){
        if(current_armor.time_stamp - armor_seq.back().time_stamp > 1){ //s
            std::cout<<"reset!!"<<std::endl;
            inited = false;
        }
    }
    armor_seq.push_back(current_armor);
    if(armor_seq.size()>200){armor_seq.erase(armor_seq.begin());}
}