#ifndef NCC_H
#define NCC_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <stdio.h>
 
class NCCTracker
{
public:
    inline void init(cv::Mat & img, cv::Rect rect);

    inline cv::Rect track(cv::Mat img);


private:
    cv::Point2f p_position;

    cv::Size p_size;

    float p_window;

    cv::Mat p_template;
};

#endif
