#ifndef _ARMOR__
#define _ARMOR__

#include <iostream>
#include <opencv2/opencv.hpp>

struct RobotInfo{
    char color;
    double ptz_pitch;
    double ptz_yaw;
    double bullet_speed;
};

struct SendInfo{

};
enum ArmorType{
    SMALL = 0,
    BIG
};

struct ArmorInfo{
    ArmorType type;
    cv::Point2f plu;
    cv::Point2f pld;
    cv::Point2f rd;
    cv::Point2f ru;
    cv::RotatedRect rect;
};
class Armor{

public:
    Armor()=default;

private:
    double time_stamp;
    cv::Point3f cam_point_;
    cv::Point3f world_point_;
    int8_t id;
    ArmorType armor_size;
};

#endif