#ifndef TCBT_H
#define TCBT_H

#include "btracker.h"
#include "CBT/cbt.h"

class tCBT : public BTracker
{
public:
    tCBT();

    void run();

    void init(cv::Mat& image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track();
    void update();
    void newFrame(cv::Mat& image, std::vector<float> predictRect);

private:
    cv::Mat currentFrame;
    std::vector<float> currentPredictRect;
    CBT cbt;
};

#endif // TCBT_H
