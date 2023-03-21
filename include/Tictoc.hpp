#pragma once
#ifndef FPS_COUNT_HPP
#define FPD_COUNT_HPP

#include <iostream>
#include <chrono>
#include <fmt/core.h>
#include <thread>
class Tictok{
public:
    explicit Tictok(){
        start_time = std::chrono::steady_clock::now();
        fps = fps_count = 0;
    };
    double this_time(){
        auto t1 = std::chrono::steady_clock::now();
        double time_stamp = std::chrono::duration<double, std::milli>(t1 - start_time).count();//ms
        return time_stamp;
    };

    void fps_calculate(int &final_fps){
//        std::cout<<"fps calculate"<<std::endl;
        fps++;
        auto time_ = this_time();
        if (time_ - fps_count >= 1000) {
            final_fps = fps;
//            fmt::print("thread process  FPS : {}",fps);
//            std::cout<< std::this_thread::get_id()<<" "<< "process FPS : " <<fps<<std::endl;
            fps = 0;
            fps_count = time_;
        }

    }

private:

    int fps, fps_count;
    std::chrono::time_point<std::chrono::steady_clock> start_time;
};

#endif