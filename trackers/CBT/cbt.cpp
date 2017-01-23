#include "cbt.h"

CBT::CBT()
{

}

void CBT::init(cv::Mat &image, cv::Rect rect, bool findConsensus){
    cv::Mat roi = image(assertRoi(rect, image.size()));
    avaliation.init(roi);
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    lastPosition = rect;
    lastImage = image;
    lastGrayImage = grayImage;
    this->findConsensus = findConsensus;
}


double CBT::track(cv::Mat &image, float &scale){
    cv::Mat mask = cv::Mat::zeros(lastImage.size(), CV_8UC1);
    cv::Rect r = assertRoi(lastPosition, image.size());
    if(r.width > 0 && r.height > 0){
        cv::Mat roi = mask(r);
        roi.setTo(255);
    }

    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);

    std::vector<float> error;
    std::vector<unsigned char> status;

    std::vector<cv::Point2f> prevPoints, nextPoints;

    cv::goodFeaturesToTrack(grayImage, prevPoints, MAXFEATURES, 0.05, 2, mask);
    double confidence = 0;
    scale = 1;
    if(prevPoints.size() > 20){
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
        if(foundRate > 0.50){
            float rotation;
            std::vector<cv::Point2f> inliers;
            cv::Point2f center;
            consensus.initialize(prevPoints);
            consensus.estimateScaleRotation(nextPoints, scale, rotation);
            lastPosition.height = round(scale * lastPosition.height);
            lastPosition.width  = round(scale * lastPosition.width);

            if(findConsensus){
                consensus.findConsensus(nextPoints, scale, rotation, center, inliers);

                // Compute new position
                lastPosition.x = round(center.x + lastPosition.x);
                lastPosition.y = round(center.y + lastPosition.y);

                cv::Rect r = assertRoi(lastPosition, image.size());
                if(r.width > 0 && r.height > 0){
                    cv::Mat roi = image(r);
                    double colorConfidence = avaliation.compare(roi);
                    confidence = foundRate * colorConfidence;
                }
            } else {
                confidence = foundRate;
            }


        }
    }

    lastImage = image;
    lastGrayImage = grayImage;
    return confidence;
}

void CBT::update(cv::Mat &image){
    cv::Rect r = assertRoi(lastPosition, image.size());
    if(r.width > 0 && r.height > 0){
        cv::Mat roi = image(r);
        avaliation.update(roi);
    }
}

cv::Rect CBT::assertRoi(cv::Rect rect, cv::Size imSize){
    cv::Rect roi = rect;
    if(roi.x < 0){
        roi.width -= roi.x;
        roi.x = 0;
    }
    if(roi.y < 0){
        roi.height -= roi.y;
        roi.y = 0;
    }
    if(roi.x+roi.width >= imSize.width)
        roi.width = imSize.width - roi.x - 2;
    if(roi.y+roi.height >= imSize.height)
        roi.height = imSize.height - roi.y- 2;

    return roi;
}
