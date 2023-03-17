////
//// Created by quonone on 23-2-22.
////
//
//#ifndef ARES_CV_PREDICTOR_H
//#define ARES_CV_PREDICTOR_H
//
//
//#include <eigen3/Eigen/Eigen>
//#include <opencv2/opencv.hpp>
//#include <eigen3/Eigen/Eigen>
//#include "../../include/data_type.h"
//#include "kalmanFilter.hpp"
//#include "anglesolver.hpp"
//
//class Predictor {
//public:
//    explicit Predictor(const std::string& params_path);
//    ~Predictor()=default;
//
//    void reset();
//    cv::Point3f predict(const Armor& armor);
//
//private:
//
//    float evaluate_threshold = 1000.0;
//    const std::string& file_path_;
//    bool inited = false;
//    std::vector<Armor> armor_seq;
//    Armor current_armor;
//    std::shared_ptr<KalmanFilter> kf;
//
//
//
//
//};
//
//
//#endif //ARES_CV_PREDICTOR_H
