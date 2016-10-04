#include "tkcf.h"

tKCF::tKCF()
{

}

void tKCF::init(cv::Mat& image, cv::Rect region){
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    cv::Mat kcfPatch = adj.init(grayImage, region);
    kcf.init(kcfPatch);
    ratio = (float)region.height/(float)region.width;
    this->region = region;
    updateModel = false;
}

void tKCF::correctState(std::vector<float> st){
    this->state = st;
    this->region.height = st[2]*ratio;
    this->region.width = st[2];
    this->region.x = st[0] - st[2]/2.0;
    this->region.y = st[1] - (st[2]*ratio/2.0);
}

void tKCF::track(){
    cv::Point2f motion;
    cv::cvtColor(currentFrame, grayImage, CV_BGR2GRAY);
    cv::Mat kcfPatch = adj.getRect(grayImage, region);
    kcf.track(kcfPatch);
    motion = kcf.getError();
    motion.x *= adj.ratio[0];
    motion.y *= adj.ratio[1];

    region.x += motion.x;
    region.y += motion.y;

    state.clear();
    state.push_back((float)region.x + (float)region.width/2.0);
    state.push_back((float)region.y + (float)region.height/2.0);
    state.push_back(region.width);

    this->stateUncertainty.clear();
    float penalityKCF = pow(DIST_ADJ*fabs(state[0] - currentPredictRect[0])/((double)region.width),2)  + pow(DIST_ADJ*fabs(state[1] - currentPredictRect[1])/((double)region.height), 2);
    float uncertainty = 1e-4*exp(-3.5*(1.1*kcf.correlation - penalityKCF));
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty*5.0);
}

void tKCF::update(){
    cv::Mat kcfPatch = adj.getRect(grayImage, region);
    kcf.updateKernel(kcfPatch);
}

void tKCF::run(){
    if(updateModel){
        update();
    } else {
        track();
    }
}

void tKCF::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}
