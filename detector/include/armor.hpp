#ifndef _ARMOR__
#define _ARMOR__

#include <iostream>
#include <opencv2/opencv.hpp>

enum ArmorType{
    SMALL = 0,
    BIG
};

struct ArmorInfo{
    ArmorType type;
    cv::Point2f center;
    cv::Point2f pl;
    cv::Point2f pu;
    cv::Point2f pr;
    cv::Point2f pd;
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