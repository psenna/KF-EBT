#ifndef FLOW_H
#define FLOW_H

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "consensus/Consensus.h"
#include <vector>
#include <cstdlib>


#define MINSIZE 40.0
#define MAXFEATURES 60
#define MINFEATURES 60
#define MIN_FEATURES_FOUND 8
#define KCF_PADDING 1.5
#define MAX_ERROR 3
#define ACCEPT_RATE 0.15


class Flow
{
public:
    Flow();
    void init(cv::Mat image, cv::Mat hsvImage, std::vector<cv::Point2f> rectangle, cv::Mat& Result);
    void getTransform(cv::Mat image, cv::Mat hsvImage, cv::Mat& Result);
    std::vector<cv::Point2f> revertTransform(cv::Point2f correction, bool mosseStatus);
    std::vector<cv::Point2f> mosseSaveTheDay(cv::Point2f correction);
    std::vector<cv::Point2f> getRectangle();
    cv::Mat getMosseCorrection();
    cv::Mat lastResult;
    bool problem;

private:
    void computeTransform();
    void getRTMatrix( const cv::Point2f* a, const cv::Point2f* b, int count, cv::Mat& M);
    void getRSMatrix( const cv::Point2f* a, const cv::Point2f* b, int count, cv::Mat& M);
    cv::Mat estimateRigidTransform( cv::InputArray src1, cv::InputArray src2, std::vector<int>& bestPoints);
    cv::Mat estimateRigidTransformRS( cv::InputArray src1, cv::InputArray src2, std::vector<int>& bestPoints);


    void rotateBox(std::vector<cv::Point2f>& box, float angle);
    void scaleBox(std::vector<cv::Point2f>& box, float scale);
    void translateBox(std::vector<cv::Point2f>& box, cv::Point2f distance);
    cv::Point2f boxCenter(std::vector<cv::Point2f> box);

    cv::Size size;
    cv::Size sizeColor;
    cv::Mat lastImage;
    cv::Mat lastHSVImage;
    cv::Mat transformMatrix;
    cv::Mat inverseTransformMatrix;
    std::vector<cv::Point2f> Lastrectangle;
    std::vector<cv::Point2f> oldRectangle;

    std::vector<cv::Point2f> bases;

    std::vector<cv::Point2f> lastKeypoints;
    std::vector<cv::Point2f> lastNoermedKeyPoints;

    cv::Mat realibleImage;
    cv::Mat realibleHSVImage;
    std::vector<cv::Point2f> realibleRectangle;
    std::vector<cv::Point2f> realibleKeipoints;
    std::vector<cv::Point2f> errorRectangle;
    std::vector<cv::Point2f> realibleNoermedPoints;

    cv::Point2f dstObj[3];
    cv::Point2f dstHSV[3];

    cv::Point2f lastCenter;
    cv::Point2f speed;

    std::vector<cv::Point2f> prevPoints, nextPoints, normedPoints;
};

#endif // FLOW_H
