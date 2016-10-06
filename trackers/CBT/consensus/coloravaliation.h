#ifndef COLORAVALIATION_H
#define COLORAVALIATION_H

#include <opencv2/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"

#define BINS 16
#define ADAPTATION 0.03
class ColorAvaliation
{
public:
    ColorAvaliation();
    void init(cv::Mat& image);
    float compare(cv::Mat& image);
    void update(cv::Mat& image);

private:
    cv::Mat histogran;
    cv::Mat model;

};

#endif // COLORAVALIATION_H
