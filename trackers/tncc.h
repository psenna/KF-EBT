#ifndef TNCC_H
#define TNCC_H

#include "btracker.h"
#include "NCC/ncc.h"

class tncc : public BTracker
{
public:
    tncc(float dist_adj = DIST_ADJ, float conf_adj = 1.1);

    void init(cv::Mat& image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track();
    void update();
    void newFrame(cv::Mat& image, std::vector<float> predictRect);
    cv::Rect getRect();

private:

    NCCTracker ncc;
    cv::Rect region;
    cv::Mat grayImage;
    cv::Mat currentFrame;
    std::vector<float> currentPredictRect;
};

#endif // TNCC_H
