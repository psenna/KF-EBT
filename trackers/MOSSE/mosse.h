#ifndef MOSSE_H
#define MOSSE_H

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/utility.hpp"


#include <ctime>
#include <cstdlib>

#define EPS 1e-20
#define RATE 0.125

class Mosse
{
public:
    Mosse();
    void init(cv::Mat frame, cv::Rect rect);
    void update(cv::Mat frame);
    cv::Point2f getPosition();
    void setPosition(cv::Rect);
    cv::Rect getRect();

    // when the target is deliverid to the mosse, use with updateTarget
    void init(cv::Mat target);
    float updateTarget(cv::Mat target, bool accept);
    void updateKernel(cv::Mat target);
    cv::Point2f error();
    bool ok;
    bool status;

private:
    void correlate(cv::Mat image, cv::Mat& resp, cv::Point2f& position, float& psr, double& max);
    void rndWarp(cv::Mat input, cv::Mat& output);
    void divSpec(cv::Mat A, cv::Mat B, cv::Mat& C);
    void preProcess(cv::Mat input, cv::Mat& output);
    void updateKernel();
    cv::Mat conjug(cv::Mat M);

    cv::Mat H, H1, H2;
    cv::Mat G;
    cv::Mat lastImage;
    cv::Point2f position;
    cv::Size size;
    cv::Mat win;
};

#endif // MOSSE_H
