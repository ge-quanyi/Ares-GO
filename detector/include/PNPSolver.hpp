#pragma once
#ifndef __PNP_
#define __PNP_
#include <opencv2/opencv.hpp>
#include "armor.hpp"

class PNPSolver{
public:

    explicit PNPSolver(const std::array<double, 9>& camera_matrix, const std::vector<double>& distortion):
    K_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()), \
    D_(cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion.data())).clone()){
        cx = camera_matrix.at(2);
        cy = camera_matrix.at(5);
        fx = camera_matrix.at(0);
        fy = camera_matrix.at(4);
    }

    cv::Point3f get_cam_point(ArmorInfo& armor){
        object_corners.clear();
        image_points.clear();
        cv::Point3f tmp_point;
        if(armor.type == SMALL){
            for (int i = 0; i < 5; i++){
                tmp_point = {armor_small_pt[i][0],armor_small_pt[i][1],0};
                object_corners.push_back(tmp_point);
            }
        }else{
            for(int i=0; i<5; i++){
                tmp_point = {armor_big_pt[i][0],armor_big_pt[i][1], 0};
                object_corners.push_back(tmp_point);
            }
        }
        image_points = {armor.center, armor.pr, armor.pu, armor.pl, armor.pd};

        cv::Mat rvec, tvec;
        cv::solvePnP(cv::Mat(object_corners), cv::Mat(image_points), K_, D_, rvec, tvec, false);
        double Z = tvec.at<double>(2, 0);
//    double X = tvec.at<double>(0, 0);
//    double Y = tvec.at<double>(1, 0);
        double X = (armor.center.x - cx) * Z / fx ;
        double Y = (armor.center.y- cy) * Z / fy ;
        return cv::Point3f(X, Y, Z);

    }
    

private:

    const cv::Mat K_;
    const cv::Mat D_;
    float fx, fy, cx, cy;

    std::vector<cv::Point3f> object_corners;
    std::vector<cv::Point2f> image_points;
    const float armor_small_pt[5][2] = {
        0.0, 0.0,
        0.0675, 0.0,
        0.0, 0.0670,
        -0.0675, 0.0,
        0.0, -0.0670,
    };

    const float armor_big_pt[5][2] = {
        0.0, 0.0,       //世界坐标系原点
        0.1125, 0.0,    //右
        0.0, 0.1120,    //上
        -0.1125, 0.0,   //左
        0.0, -0.1120,   //下
    };

    

};





#endif
