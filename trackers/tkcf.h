#ifndef TKCF_H
#define TKCF_H

#include "btracker.h"
#include "kcf/kcf.h"
#include "kcf/adjust.h"

class tKCF : public BTracker
{
public:
    tKCF(float dist_adj = DIST_ADJ, float conf_adj = 1.1);

    void init(cv::Mat& image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track();
    void update();
    void newFrame(cv::Mat& image, std::vector<float> predictRect);
    cv::Rect getRect();

private:
    KCF_Tracker kcf;
    Adjust adj;
    float updateRatio;
    cv::Rect region;
    cv::Mat grayImage;
    cv::Mat currentFrame;
    std::vector<float> currentPredictRect;
};

#endif // TKCF_H
