#include "tmosse.h"

tMosse::tMosse()
{

}

void tMosse::init(cv::Mat& image, cv::Rect region){
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    cv::Mat mossePatch = adj.init(grayImage, region);
    mosse.init(mossePatch);
    ratio = (float)region.height/(float)region.width;
    this->region = region;
    updateModel = false;
}

void tMosse::correctState(std::vector<float> st){
    this->state = st;
    this->region.height = round(st[2]*ratio);
    this->region.width = round(st[2]);
    this->region.x = round(st[0] - st[2]/2.0);
    this->region.y = round(st[1] - (st[2]*ratio/2.0));
}

void tMosse::track(){
    cv::Point2f motion;
    cv::cvtColor(currentFrame, grayImage, CV_BGR2GRAY);
    cv::Mat mossePatch = adj.getRect(grayImage, region);
    double confidence = mosse.updateTarget(mossePatch, false);
    motion = mosse.error();

    region.x += motion.x;
    region.y += motion.y;

    state.clear();
    state.push_back(round((float)region.x + (float)region.width/2.0));
    state.push_back(round((float)region.y + (float)region.height/2.0));
    state.push_back(region.width);

    stateUncertainty.clear();
    float penalityMosse = pow(DIST_ADJ*fabs(state[0] - currentPredictRect[0])/((double)region.width),2)  +
            pow(DIST_ADJ*fabs(state[1] - currentPredictRect[1])/((double)region.height), 2);

    float uncertainty = 1e-4*exp(-3.5*(1.0*confidence - penalityMosse));
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty*5.0);
}

void tMosse::update(){
    cv::Mat mossePatch = adj.getRect(grayImage, region);
    mosse.updateKernel(mossePatch);
}

void tMosse::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}

cv::Rect tMosse::getRect(){
    return cv::Rect();
}
