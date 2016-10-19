#ifndef ADJUST_H
#define ADJUST_H

#include <opencv2/opencv.hpp>

#define MINSIZE 44
#define KCF_PADDING 1.5

class Adjust
{
public:
    Adjust();
    cv::Mat init(cv::Mat& image, cv::Rect rect);
    cv::Mat getRect(cv::Mat& image, cv::Rect rect);

    cv::Point2f srcTri[3], dstObj[3];


    cv::Size size;
    float ratio[2];
};

#endif // ADJUST_H
