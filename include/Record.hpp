//
// Created by quanyi on 22-5-18.
//

#pragma once

#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <chrono>

class Record{
public:
    Record(){
        start_time = std::chrono::steady_clock::now();
        setParam();
    }
    ~Record(){

        delete videoWriter;
    }

    bool writeVideo(cv::Mat src){

        if(!is_inited){
            videoWriter->open(path, CV_FOURCC('M','J','P','G'), fps, size, true);
            is_inited = true;
            record_time = start_time;
        }

        auto this_time = std::chrono::steady_clock::now();
        double duration = std::chrono::duration<double,milli>(this_time-record_time).count();
        if(duration>1000/fps){
            videoWriter->write(src);
            record_time = this_time;
        }
    }
    bool setParam(){
        time_t tt;
        time(&tt);
        tt = tt + 8*3600;
        tm* t = gmtime(&tt);
        std::string filename  = std::to_string(t->tm_mon+1) +"-"+
                                std::to_string(t->tm_mday)+"-"+
                                std::to_string(t->tm_hour)+
                                std::to_string(t->tm_min)+
                                std::to_string(t->tm_sec);
        //std::cout<<"t "<<filename<<std::endl;
        path = "../video/"+filename + ".avi";
        //std::cout<<"path is "<<path<<std::endl;
        fps = 30;
        size = cv::Size(960,768);
        is_inited = false;
        videoWriter = new cv::VideoWriter();
    }

private:
    std::string path;
    int code;
    double fps;
    cv::Size size;
    bool is_inited;
    cv::VideoWriter * videoWriter;
    int record_cnt;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
    std::chrono::time_point<std::chrono::steady_clock> record_time;

};
