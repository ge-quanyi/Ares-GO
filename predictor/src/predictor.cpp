////
//// Created by quonone on 23-2-22.
////
//
//#include "../include/predictor.h"
//
//Predictor::Predictor(const std::string &params_path):file_path_(params_path) {
//    cv::FileStorage fs(file_path_, cv::FileStorage::READ);
//    if(!fs.isOpened()){
//        std::cout<<"[ERROR ]predictor open params file error, please check \r\n";
//        exit(-1);
//    }
//    Eigen::MatrixXd A(6,6);
//    Eigen::MatrixXd P0(6,6);
//    Eigen::MatrixXd H(3,6);
//    Eigen::MatrixXd Q(6,6);
//    Eigen::MatrixXd R(3,3);
//    Eigen::MatrixXd I(6,6);
//    A<< 1,0,0,0.5,0,0,
//        0,1,0,0,0.5,0,
//        0,0,1,0,0,0.5,
//        0,0,0,1,0,0,
//        0,0,0,0,1,0,
//        0,0,0,0,0,1;
//    P0.setIdentity(6,6);
//    I.setIdentity(6,6);
//    H << 1,0,0,0,0,0,
//         0,1,0,0,0,0,
//         0,0,1,0,0,0;
//    Q << fs["sentry_up"]["Q00"], 0,0,0,0,0,
//         0,fs["sentry_up"]["Q11"],0,0,0,0,
//         0,0,fs["sentry_up"]["Q22"],0,0,0,
//         0,0,0,fs["sentry_up"]["Q33"],0,0,
//         0,0,0,0,fs["sentry_up"]["Q44"],0,
//         0,0,0,0,0,fs["sentry_up"]["Q55"];
//    R << fs["sentry_up"]["R00"],0,0,
//         0,fs["sentry_up"]["R11"],0,
//         0,0,fs["sentry_up"]["R22"];
//
//    kf = std::make_shared<KalmanFilter>(A,P0,H,Q,R);
//    fs.release();
//}
//
//void Predictor::reset() {
//
//    Eigen::VectorXd x0(6) ;
//    x0 << current_armor.world_point_.x, current_armor.world_point_.y, current_armor.world_point_.z,0,0,0;
//    kf->init(x0, current_armor.time_stamp);
//    armor_seq.clear();
//}
//
////
//cv::Point3f Predictor::predict(const Armor &armor) {
//    current_armor = armor;
//    if(!inited){
//        reset();
//        inited = true;
//    }
//    Eigen::VectorXd y0(3);
//    y0 << current_armor.world_point_.x, current_armor.world_point_.y, current_armor.world_point_.z;
//    kf->predict(y0,current_armor.time_stamp);
//    //
//    Eigen::VectorXd Ek(3); //3x1
//    Ek = y0 - kf->H*kf->X_hat_new;
//    Eigen::MatrixXd Dk(3,3);
//    Dk=kf->H*kf->P+kf->R; //3x3
//    auto rk = Ek.transpose()*Dk.inverse()*Ek;
//
//    if(rk < evaluate_threshold){
//        reset();
//        cv::Point3f result = current_armor.world_point_;
//        return result;
//    }
//
//    ///TODO : result evalue
//    kf->update(y0);
//    cv::Point3f result = cv::Point3f (kf->X_hat_new(0),kf->X_hat_new(1),kf->X_hat_new(2));
//    //todo: add fly
//    return result;
//}