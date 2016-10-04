#include "cbt.h"

CBT::CBT()
{

}

void CBT::init(cv::Mat &image, cv::Rect rect){
    cv::Mat roi = image(rect);
    avaliation.init(roi);
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    lastPosition = rect;
    lastImage = image.clone();
    lastGrayImage = grayImage.clone();
}


double CBT::track(cv::Mat &image){
    cv::Mat mask = cv::Mat::zeros(lastImage.size(), CV_8UC1);
    cv::Mat roi = mask(lastPosition);
    roi.setTo(255);
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);

    std::vector<float> error;
    std::vector<unsigned char> status;

    std::vector<cv::Point2f> prevPoints, nextPoints;

    cv::goodFeaturesToTrack(grayImage, prevPoints, MAXFEATURES, 0.001, 3, mask);
    double confidence = 0;
    if(prevPoints.size() > 0){
        std::vector<cv::Point2f> pointsBack;
        std::vector<unsigned char> status_back;
        std::vector<float> err_back;
        cv::calcOpticalFlowPyrLK(lastGrayImage, grayImage, prevPoints, nextPoints, status, error);

        cv::calcOpticalFlowPyrLK(grayImage, lastGrayImage, nextPoints, pointsBack, status_back, err_back);
        for(int i =prevPoints.size()-1; i >= 0; i--){

            float l2Error = cv::norm(prevPoints[i]-pointsBack[i]);

            if(!status[i] || !status_back[i] || l2Error > MAX_ERROR){
                nextPoints.erase(nextPoints.begin() + i);
                prevPoints.erase(prevPoints.begin() + i);
            }
        }

        float foundRate = (float)nextPoints.size()/(float)pointsBack.size();
        float scale, rotation;
        std::vector<cv::Point2f> inliers;
        cv::Point2f center;
        consensus.initialize(prevPoints);
        consensus.estimateScaleRotation(nextPoints, scale, rotation);
        consensus.findConsensus(nextPoints, scale, rotation, center, inliers);

        // Compute new position
        lastPosition.height *= scale;
        lastPosition.width  *= scale;
        lastPosition.x += center.x;
        lastPosition.y += center.y;

        roi = image(lastPosition);
        double colorConfidence = avaliation.compare(roi);
        confidence = foundRate * colorConfidence;
    }

    lastImage = image.clone();
    lastGrayImage = grayImage.clone();
    return confidence;
}

void CBT::update(cv::Mat &image){
    cv::Mat roi = image(lastPosition);
    avaliation.update(roi);
}
