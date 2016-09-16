#ifndef TASMS_H
#define TASMS_H

#include "btracker.h"
#include "ASMS/colotracker.h"

class tASMS : public BTracker
{
public:
    tASMS();

    void run();

    void init(cv::Mat image, cv::Rect region);
    void correctState(std::vector<float> st);
    void track(cv::Mat image);

private:
    ColorTracker asms;

};

#endif // TASMS_H
