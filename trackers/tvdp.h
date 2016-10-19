#ifndef TVDP_H
#define TVDP_H

#include "btracker.h"
#include "CBT/cbt.h"


class tVDP : public BTracker
{
public:
    tVDP();

    void run();

    void init(cv::Mat& image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track();
    void update();
    void newFrame(cv::Mat& image, std::vector<float> predictRect);
    cv::Rect getRect();

private:
    cv::Mat currentFrame;
    std::vector<float> currentPredictRect;
    CBT cbt;
    float initialWidth;
};

#endif // TVDP_H
