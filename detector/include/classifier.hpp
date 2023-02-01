#ifndef __CLASS_
#define __CLASS_
#include <iostream>
#include <opencv2/opencv.hpp>


class Classifier{
public:
    explicit Classifier()=default;

    int number_check(cv::Mat& src);


private:
    std::string modle_file_;


};

#endif