//
// Created by quonone on 23-1-31.
//

#ifndef ARES_CV_ARMOR_DETECT_H
#define ARES_CV_ARMOR_DETECT_H

#include <opencv2/opencv.hpp>
#include "PNPSolver.hpp"
#include "Tictoc.hpp"
#include "../include/ovinference.h"
#include "../../predictor/include/EKFpredictor.h"
#include "../../serial/serial.h"
#include "anglesolver.hpp"
#include "Message.hpp"

typedef std::pair<double, cv::Mat> Camdata;
extern Publisher<Camdata> cam_publisher;
extern Publisher<RobotInfo> serial_publisher;

class ArmorDetect{
public:
    explicit ArmorDetect();
    ~ArmorDetect()=default;

    void run();
private:
    std::shared_ptr<PNPSolver> pnpsolver;
    std::unique_ptr<Tictok> tic;
    std::shared_ptr<OvInference> ovinfer;
    std::shared_ptr<EKFPredictor> predictor;
    std::shared_ptr<AngleSolver> as;
    int8_t locked_id = -1;
    int lose_cnt = 0;
    int lock_cnt = 0;
    //zmq value define
    void *context;
    void *publisher;
    int bind;

    double pitch_last,yaw_last,dis_last,id_last;

    void color_check(const char color, std::vector<OvInference::Detection>& results);
    void armor_sort(OvInference::Detection& final_obj, std::vector<OvInference::Detection>& results, cv::Mat& src);
    void draw_target(const OvInference::Detection& obj, cv::Mat& src);

};
#endif //ARES_CV_ARMOR_DETECT_H
