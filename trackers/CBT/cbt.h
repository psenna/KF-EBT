#ifndef CBT_H
#define CBT_H

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <vector>
#include <cstdlib>
#include <iostream>
#include <cmath>

#include "consensus/Consensus.h"
#include "consensus/coloravaliation.h"

#define MAXFEATURES 60
#define MAX_ERROR 3

class CBT
{
public:
    CBT();
    void init(cv::Mat& image, cv::Rect rect, bool findConsensus);
    double track(cv::Mat &image, float &scale);
    void update(cv::Mat &image);

    cv::Rect lastPosition;
private:
    ColorAvaliation avaliation;
    cmt::Consensus consensus;
    cv::Mat lastImage;
    cv::Mat lastGrayImage;
    bool findConsensus = true;
    cv::Rect assertRoi(cv::Rect rect, cv::Size imSize);

};

#endif // CBT_H
