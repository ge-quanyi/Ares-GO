//
// Created by quonone on 23-1-31.
//

#ifndef ARES_CV_ARMOR_DETECT_H
#define ARES_CV_ARMOR_DETECT_H

#include <opencv2/opencv.hpp>
#include "PNPSolver.hpp"
#include "Tictoc.hpp"
#include "../camera/include/camera.h"
#include "../include/ovinference.h"
#include "../serial/serial.h"
#include "../../predictor/include/EKFpredictor.h"
#include "anglesolver.hpp"

extern std::shared_ptr<Camera> camera;
extern std::shared_ptr<SerialPort> serial;

class ArmorDetect{
public:
    explicit ArmorDetect();
    ~ArmorDetect()=default;

    void run();
    bool getdata(cv::Mat& src);
private:
    std::shared_ptr<PNPSolver> pnpsolver;
    std::unique_ptr<Tictok> tic;
    std::shared_ptr<OvInference> ovinfer;
    std::shared_ptr<EKFPredictor> predictor;
    std::shared_ptr<AngleSolver> as;
    std::mutex pub_lock;
    int8_t locked_id = -1;
    int lose_cnt = 0;
    int lock_cnt = 0;

    double pitch_last,yaw_last,dis_last,id_last;

    void color_check(const char color, std::vector<OvInference::Detection>& results);
    void armor_sort(OvInference::Detection& final_obj, std::vector<OvInference::Detection>& results, cv::Mat& src);
    void draw_target(const OvInference::Detection& obj, cv::Mat& src);

    void putdata(const cv::Mat& img);
    std::queue<cv::Mat> image_to_display_;
};
#endif //ARES_CV_ARMOR_DETECT_H
