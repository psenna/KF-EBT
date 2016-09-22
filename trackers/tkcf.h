#ifndef TKCF_H
#define TKCF_H

#include "btracker.h"
#include "kcf/kcf.h"
#include "kcf/adjust.h"

class tKCF : public BTracker
{
public:
    tKCF();

    void run();

    void init(cv::Mat& image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track(cv::Mat& image, std::vector<float> predictRect);
    void update(cv::Mat& image);

private:
    KCF_Tracker kcf;
    Adjust adj;
    cv::Rect region;
    cv::Mat grayImage;
};

#endif // TKCF_H
