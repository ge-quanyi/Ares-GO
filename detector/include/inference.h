#pragma once
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <inference_engine.hpp>


using namespace std;
using namespace cv;
using namespace InferenceEngine;

//color : red 1  blue 0   gray 2
struct ArmorObject
{
    Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;  //lu ld rd ru
};

struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};

class Inference
{
public:
    Inference(const std::string& path);
    ~Inference();
    bool detect(Mat &src,vector<ArmorObject>& objects);
    bool initModel();
private:

    Core ie;
    CNNNetwork network;                // 网络
    ExecutableNetwork executable_network;       // 可执行网络
    InferRequest infer_request;      // 推理请求
    MemoryBlob::CPtr moutput;
    string input_name;
    string output_name;

    const std::string& model_path_;
    Eigen::Matrix<float,3,3> transfrom_matrix;

    inline float calcTriangleArea(cv::Point2f pts[3])
    {
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

        auto p = (a + b + c) / 2.f;

        return sqrt(p * (p - a) * (p - b) * (p - c));
    }


    inline float calcTetragonArea(cv::Point2f pts[4])
    {
        return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
    }
};
