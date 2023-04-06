#include "../include/EKFpredictor.h"
#include "glog/logging.h"

EKFPredictor::EKFPredictor() {
    cv::FileStorage fin("../params/params.yaml", cv::FileStorage::READ);
    if(!fin.isOpened()){
        std::cout<<"[ERROR ]predictor open params file error, please check \r\n";
        exit(-1);
    }
    ekf = std::make_shared<AdaptiveEKF<5,3>>();

    fin["sentry_up"]["Q00"] >> ekf->Q(0, 0);
    fin["sentry_up"]["Q11"] >> ekf->Q(1, 1);
    fin["sentry_up"]["Q22"] >> ekf->Q(2, 2);
    fin["sentry_up"]["Q33"] >> ekf->Q(3, 3);
    fin["sentry_up"]["Q44"] >> ekf->Q(4, 4);
    // 观测过程协方差
    fin["sentry_up"]["R00"] >> ekf->R(0, 0);
    fin["sentry_up"]["R11"] >> ekf->R(1, 1);
    fin["sentry_up"]["R22"] >> ekf->R(2, 2);
    std::cout<<"QR matrix: "<<ekf->Q <<"\r\n "<<ekf->R<<std::endl;
    fin.release();
    anglesolver = std::make_unique<AngleSolver>();
}

void EKFPredictor::read_params() {

}
void EKFPredictor::reset() {

    last_dis = current_armor.distance;
    Eigen::Matrix<double, 5, 1> Xr;
    Xr << current_armor.world_point_.x, 0, current_armor.world_point_.y, 0, current_armor.world_point_.z;
    ekf->init(Xr);
    armor_seq.clear();
    ekf->P.setIdentity();
//    std::cout<<" reset "<<"\r\n";
    LOG(WARNING) << "ekf reset ";
}

void EKFPredictor::predict(const Armor& armor,  cv::Point3f& cam_pred,const RobotInfo& robot) {
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
        last_time = current_armor.time_stamp;
        inited = true;
    }

    last_dis = current_armor.distance;

    double delta_t = current_armor.time_stamp - last_time;
//    std::cout<<"delta t "<<delta_t<<std::endl;
    last_time = current_armor.time_stamp;

    Predict predictfunc;
    Measure measure;

    Eigen::Matrix<double, 5, 1> Xr;
    Xr << current_armor.world_point_.x, 0, current_armor.world_point_.y, 0, current_armor.world_point_.z; //input

    Eigen::Matrix<double, 3, 1> Yr;
    measure(Xr.data(), Yr.data());  //convert xyz to pitch yaw

    predictfunc.delta_t = delta_t;
    ekf->predict(predictfunc);//predict

    Eigen::Matrix<double, 5, 1> Xe = ekf->update(measure, Yr);//best evalute
    double value = ekf->estimate();
    if(value>100){
        inited = false;
    }
    LOG(INFO)<<"ekf estimate "<< value;
    std::cout<<"ekf estimate "<<value<<"\r\n";
    double predict_time = current_armor.distance/robot_.bullet_speed + 0.01;
    std::cout<<"predict time "<<predict_time<<"b speed "<<robot_.bullet_speed<<"\n";

    predictfunc.delta_t = predict_time;//use measure speed to predict next
    Eigen::Matrix<double, 5, 1> Xp;

    predictfunc(Xe.data(), Xp.data()); //use ekf v to predict next position
    Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)};

    auto result = cv::Point3f(p_pw(0,0),p_pw(1,0),p_pw(2,0));
    cv::Point3f abs_pred;
    //get final abs
    motion2abs(result, abs_pred);

    // Eigen::VectorXd Ek(3); //3x1
    // Ek = y0 - kf->H*kf->X_hat_new;
    // Eigen::MatrixXd Dk(3,3);
    // Dk=kf->H*kf->P+kf->R; //3x3
    // auto rk = Ek.transpose()*Dk.inverse()*Ek;
    // std::cout<<"kafang "<<rk<<std::endl;

    // if(rk < evaluate_threshold){
    //     reset();
    //     cv::Point3f result = current_armor.world_point_;
    //     return result;
    // }


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