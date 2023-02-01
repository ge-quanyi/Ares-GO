#ifndef _BASE_DETECTOR__
#define _BASE_DETECTOR__

#include <opencv2/opencv.hpp>
#include <fmt/core.h>
#include <fmt/color.h>
#include "armor.hpp"

class BaseDetector {

public:
    explicit BaseDetector();
    ~BaseDetector()=default;
 
    void image_pretreatment(cv::Mat& src, int8_t team);
    bool find_lignt(cv::Mat& brinary_img, cv::Mat& src); 
    bool match_armor(std::vector<cv::RotatedRect>&lights);
    ArmorInfo& armor_detect(cv::Mat& src, int8_t team);
    
    cv::Mat dst;

private:

    std::vector<cv::Mat> split_src_;
    cv::Mat gray_src_ ;
    cv::Mat splited_src_;

    std::vector<cv::RotatedRect> LightBars;
    std::vector<ArmorInfo> tmp_armor_seq;
    ArmorInfo final_armor;

    int gray_threshold;
    int red_threshold;
    int blue_threshold;
    int light_size_min;

    const cv::Mat element7_9 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7,9));

    inline void correct_light_wh(float& width, float&height){
        if(width > height){
            float tmp = height;
            height = width;
            width = tmp;
        }
    }

    inline double get_dis_2d(cv::Point2f& p1, cv::Point2f& p2){
        return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
    }

    inline void draw_tmp_object(cv::Mat& src){
        if(tmp_armor_seq.size()){
            for(const auto &armor:tmp_armor_seq){
                auto rect = armor.rect;
                cv::Point2f point[4];
                rect.points(point);
                for (int i = 0; i < 4; i++) {
                    line(src, point[i], point[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
            }
        }
    }

    inline void draw_final_object(cv::Mat& src){
        auto rect = final_armor.rect;
        cv::Point2f point[4];
        for(int i=0; i<4; i++){
            line(src, point[i], point[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }

};




#endif