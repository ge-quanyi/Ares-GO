#pragma once
#ifndef __PNP_
#define __PNP_
#include <opencv2/opencv.hpp>
#include "ovinference.h"
#include "inference.h"
class PNPSolver{
public:

    explicit PNPSolver(const std::string& file_path):file_path_(file_path){

        cv::FileStorage fs(file_path_, cv::FileStorage::READ);
        if(!fs.isOpened()){
            std::cout<<"[ERROR ] open camera params file failed! "<<std::endl;
            exit(-1);
        }

        fs["camera_matrix"]>>K_;
        fs["distortion_coefficients"]>>D_;
        fs.release();
        std::cout<<"k "<<K_<<std::endl;
        std::cout<<"d "<<D_<<std::endl;
    }

    inline cv::Point3f get_cam_point(const ArmorObject& obj){
        if(obj.cls<=0)
            return cv::Point3f (0,0,0);
        object_corners.clear();
        image_points.clear();
        cv::Point3f tmp_point;
        // big armor
        //todo infantry3
        if(obj.cls==1){
            for (int i = 0; i < 4; i++){
                tmp_point = {armor_big_pt[i][0],armor_big_pt[i][1],0};
                object_corners.push_back(tmp_point);
            }
        }else{
            for(int i=0; i<4; i++){
                tmp_point = {armor_small_pt[i][0],armor_small_pt[i][1], 0};
                object_corners.push_back(tmp_point);
            }
        }
        image_points = {obj.apex[0],obj.apex[1],obj.apex[2],obj.apex[3]};

        cv::Mat rvec, tvec;
        cv::solvePnP(cv::Mat(object_corners), cv::Mat(image_points), K_, D_, rvec, tvec, false);
        double X = tvec.at<double>(2, 0);
        double Y = -tvec.at<double>(0, 0);
        double Z = -tvec.at<double>(1, 0);
        return cv::Point3f(X, Y, Z);

    }
    

private:

    cv::Mat K_;
    cv::Mat D_;
    const std::string& file_path_;

    std::vector<cv::Point3f> object_corners;
    std::vector<cv::Point2f> image_points;
    const float armor_small_pt[4][2] = {
        -0.0675, 0.0275,
        -0.0675, -0.0275,
        0.0675, -0.0275,
        0.0675, 0.0275,
    };

    const float armor_big_pt[4][2] = {
        -0.1125, 0.0275,    //lu
        -0.1125, -0.0275,    //上
        0.1125, -0.0275,   //左
        0.1125, 0.0275,   //下
    };

    

};





#endif
