#ifndef _ARMOR__
#define _ARMOR__

#include <iostream>
#include <opencv2/opencv.hpp>

struct RobotInfo{
    char color;
    double ptz_pitch;
    double ptz_yaw;
    double ptz_roll;
    double bullet_speed;
};


class Armor{

public:
    Armor()=default;

    double time_stamp;
    int8_t id;
    cv::Point3f cam_point_;
    cv::Point3f world_point_;
    double distance;

private:

};

#endif