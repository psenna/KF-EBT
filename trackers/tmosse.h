#ifndef TMOSSE_H
#define TMOSSE_H

#include "btracker.h"
#include "MOSSE/mosse.h"
#include "kcf/adjust.h"

class tMosse : public BTracker
{
public:
    tMosse();

    void init(cv::Mat& image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track();
    void update();
    void newFrame(cv::Mat& image, std::vector<float> predictRect);
    cv::Rect getRect();

private:
    Mosse mosse;
    Adjust adj;
    cv::Rect region;
    cv::Mat grayImage;
    cv::Mat currentFrame;
    std::vector<float> currentPredictRect;
};

#endif // TMOSSE_H
