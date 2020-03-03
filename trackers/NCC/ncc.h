#ifndef NCC_H
#define NCC_H

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
 
class NCCTracker
{
public:
    void init(cv::Mat & img, cv::Rect rect);

    double track(cv::Mat img);

    cv::Point2f p_position;

    cv::Size p_size;

    float p_window;

    cv::Mat p_template;
};

#endif //NCC_H
