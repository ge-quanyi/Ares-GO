//
// Created by quonone on 23-1-31.
//
#include "../include/detector.hpp"
void BaseDetector::image_pretreatment(cv::Mat& src, int8_t team){
    // cv::split(src, split_src_);
    // cv::cvtColor(src,gray_src_,cv::COLOR_BGR2GRAY);

    // if(team){//shoot red
    //     cv::subtract(split_src_[2],split_src_[1], splited_src_);
    //     cv::threshold(splited_src_,splited_src_,red_threshold,255 ,cv::THRESH_BINARY);
    //     cv::threshold(gray_src_, gray_src_, gray_threshold, 255, cv::THRESH_BINARY);
    //     cv::dilate(splited_src_, splited_src_, element7_9);
    // }else{
    //     cv::subtract(split_src_[0],split_src_[1], splited_src_);
    //     cv::threshold(splited_src_,splited_src_,blue_threshold,255 ,cv::THRESH_BINARY);
    //     cv::threshold(gray_src_, gray_src_, gray_threshold, 255, cv::THRESH_BINARY);
    //     cv::dilate(splited_src_, splited_src_, element7_9);
    // }

    // dst = splited_src_ & gray_src_;

    // return dst;

    cv::cvtColor(src, gray_src_,cv::COLOR_BGR2GRAY);

    cv::threshold(gray_src_, dst, gray_threshold, 255, cv::THRESH_BINARY);
}

bool BaseDetector::find_lignt(cv::Mat& brinary_img, cv::Mat& src){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> white_hierarchy;
    cv::RotatedRect lightbar;
    LightBars.clear();

    cv::findContours(brinary_img, contours, white_hierarchy,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    for(const auto& contour : contours){
        if(contour.size() < 10) continue;
        auto light_fitEllipse = cv::fitEllipse(contour);
        auto light_minAresRect = cv::minAreaRect(contour);
        lightbar.angle = light_fitEllipse.angle;
        lightbar.center = light_minAresRect.center;

        correct_light_wh(light_minAresRect.size.width,light_minAresRect.size.height);
        lightbar.size.height = light_minAresRect.size.height;
        lightbar.size.width = light_minAresRect.size.width;

        if(lightbar.size.width / lightbar.size.height > 0.7) continue;
        if((lightbar.center.x-lightbar.size.width)<0) continue;
        if((lightbar.center.y-lightbar.size.height)<0) continue;
        if((lightbar.center.x+lightbar.size.width)>src.cols) continue;
        if((lightbar.center.y+lightbar.size.height)>src.rows) continue;

        auto rect = light_minAresRect.boundingRect();
        if(rect.x>=0&&rect.width>=0&&rect.x+rect.width<=src.cols&&rect.y>=0&&rect.height>=0&&rect.y+rect.height<=src.rows){
            int b_channel_sum, r_channel_sum;
            auto roi = src(rect);

            for(int i=0; i<roi.rows; i++){
                for(int j=0; j<roi.cols;j++){
                    if(cv::pointPolygonTest(contour,cv::Point2f(rect.x+j, rect.y+i), false)>=0){
                        b_channel_sum += roi.at<cv::Vec3b>(j,i)[0];
                        r_channel_sum += roi.at<cv::Vec3b>(j,i)[2];
                    }
                }
            }

            if(b_channel_sum<r_channel_sum)//blue
                continue;
            if(lightbar.angle<45 || lightbar.angle >135){
                LightBars.emplace_back(lightbar);
            }
        }
    }

    if(LightBars.size()){
        fmt::print(fg(fmt::color::green),"[INFO] LIGHT NUM IS {} .", LightBars.size());
        return true;
    }else
        return false;

}

bool BaseDetector::match_armor(std::vector<cv::RotatedRect>&lights){
    tmp_armor_seq.clear();
    for(uint i = 0; i < lights.size()-1; i++){
        for(uint j = i+1; j< lights.size(); j++){
            double height_diff = abs(lights[i].size.height-lights[j].size.height);
            double height_sum = lights[i].size.height + lights[j].size.height;
            double width_diff = abs(lights[i].size.width - lights[j].size.height);
            double width_sum = lights[i].size.width + lights[j].size.width;
            double angle_diff = fabs(lights[i].angle - lights[j].angle);
            double Y_diff = abs(lights[i].center.y-lights[j].center.y);
            double MH_diff = (std::min(lights[i].size.height,lights[j].size.height))* 1.5;
            double height_max = std::max(lights[i].size.height, lights[j].size.height);
            double X_diff = abs(lights[i].center.x - lights[j].center.x);

            if(Y_diff<0.5*height_max &&
               X_diff<height_max*5 &&
               (angle_diff<10||180-angle_diff<10) /*&&
                height_diff / height_max < 0.6 &&
                width_diff / width_sum < 0.6 &&
                X_diff / MH_diff > 0.5*/) {

                cv::RotatedRect light_tmp;

                auto left_light = lights[i].center.x < lights[j].center.x ? lights[i] : lights[j];
                auto right_light = lights[i].center.x > lights[j].center.x ? lights[i] : lights[j];
                ArmorInfo tmp_armor;
                auto center = (left_light.center+right_light.center)*0.5;
                auto pl = left_light.center;
                auto pr = right_light.center;
                cv::Point2f tmp_pt_1(pr.x-center.x, pr.y-center.y);
                cv::Point2f tmp_pt_2(-tmp_pt_1.y, tmp_pt_1.x);
                cv::Point2f pd(tmp_pt_2.x+center.x, tmp_pt_2.y+center.y);

                cv::Point2f tmp_pt_3(pl.x-center.x, pl.y-center.y);
                cv::Point2f tmp_pt_4(-tmp_pt_3.y, tmp_pt_3.x);
                cv::Point2f pu(tmp_pt_4.x+center.x, tmp_pt_4.y+center.y);

                double width = get_dis_2d(left_light.center, right_light.center);
                double height = (left_light.size.height+right_light.size.height)*0.5;
                float wh_rate = width/height;
                auto type  = wh_rate > 3.3 ? BIG : SMALL;
                cv::RotatedRect rect;
                rect.angle = (180 - angle_diff) < 10? (left_light.angle+right_light.angle)*0.5+90 :
                             (left_light.angle+right_light.angle)*0.5;
                rect.center = center;
                rect.size.height = height;
                rect.size.width = width;
                tmp_armor = {type, center, pl, pu, pr, pd, rect};
                tmp_armor_seq.emplace_back(tmp_armor);
            }
        }
    }

    if(!tmp_armor_seq.size()) return false;
    else return true;

}


ArmorInfo& BaseDetector::armor_detect(cv::Mat& src, int8_t team){
    image_pretreatment(src, team);
    if(!find_lignt(dst, src)){
        fmt::print(fg(fmt::color::yellow),"[INFO ] NO LIGHT FOUNT..");

    }
    if(!match_armor(this->LightBars)){
        fmt::print(fg(fmt::color::yellow),"[INFO ] NO ARMOR FOUNT..");
    }


    draw_tmp_object(src);
    ///TODO: classifier

    final_armor = tmp_armor_seq[0];
    return final_armor;

}