//
// Created by quonone on 23-1-31.
//

#ifndef ARES_CV_ARMOR_DETECT_H
#define ARES_CV_ARMOR_DETECT_H

#include <opencv2/opencv.hpp>
#include "detector.hpp"
#include "classifier.hpp"
#include "PNPSolver.hpp"
#include "Tictoc.hpp"
#include "../camera/include/camera.h"
extern std::shared_ptr<Camera> camera;
class ArmorDetect{
public:
    explicit ArmorDetect();
    ~ArmorDetect()=default;

    void run();
    std::deque<cv::Mat> image_to_display_;
private:
    std::unique_ptr<BaseDetector> detector;
    std::shared_ptr<Classifier> classifier;
    std::shared_ptr<PNPSolver> pnpsolver;
    std::unique_ptr<Tictok> tic;
};
#endif //ARES_CV_ARMOR_DETECT_H
