//
// Created by quonone on 23-2-17.
//
#pragma once
#ifndef ARES_OVINFERENCE_H
#define ARES_OVINFERENCE_H

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
class OvInference{
public:
    explicit OvInference(const std::string& model_path);
    ~OvInference()=default;

    struct Obj{
        cv::Rect box; //Rect box
        cv::Point2f p1; //face points
        cv::Point2f p2;
        cv::Point2f p3;
        cv::Point2f p4;
    };
    //        哨兵  1   2   3   4   5   前哨战   基地   基地大装甲
    // blue：   0   1   2   3   4   5     6      7       8
    // red：    9   10  11  12  13  14    15     16      17
    struct Detection {
        int class_id;
        float confidence;
        Obj obj;
    };

    void infer(cv::Mat& img, std::vector<Detection>& final_result );

private:

    struct Resize {
        cv::Mat resized_image;
        int dw;
        int dh;
    };

    std::shared_ptr<ov::Model> model;
    ov::CompiledModel compiled_model;
    const std::string& model_path_;
    const float SCORE_THRESHOLD = 0.6;
    const float NMS_THRESHOLD = 0.5;
    const float CONFIDENCE_THRESHOLD = 0.6;
    Resize resize_and_pad(cv::Mat &img, cv::Size new_shape);
    void restore_scale(Detection &detection, float rx, float ry);


};
#endif //ARES_OVINFERENCE_H
