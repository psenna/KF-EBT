#include "tasms.h"

tASMS::tASMS(float dist_adj, float conf_adj)
{
    this->dist_adj = dist_adj;
    this->conf_adj = conf_adj;
}

void tASMS::run(){
    if(updateModel){
        update();
    } else {
        track();
    }
}


void tASMS::init(cv::Mat& image, cv::Rect region){
    asms.init(image, region.x, region.y, region.x + region.width, region.y + region.height);
    ratio = (float)region.height/(float)region.width;
    updateModel = false;
}

void tASMS::correctState(std::vector<float> st){
    this->state = st;
    asms.lastPosition.height = st[2]*ratio;
    asms.lastPosition.width = st[2];
    asms.lastPosition.x = st[0] - asms.lastPosition.width/2;
    asms.lastPosition.y = st[1] - asms.lastPosition.height/2;
}

void tASMS::track(){
    double confidenceASMS = 0;
    cv::Rect asmsRect;
    BBox * bb = asms.track(currentFrame, &confidenceASMS);

    if (bb != NULL){
        asmsRect = cv::Rect(bb->x + bb->width/2.0, bb->y + bb->height/2.0, bb->width, bb->height);
        delete bb;
    }

    // write state
    state.clear();
    state.push_back(asms.lastPosition.x + asms.lastPosition.width/2);
    state.push_back(asms.lastPosition.y + asms.lastPosition.height/2);
    state.push_back(asms.lastPosition.width);

    this->stateUncertainty.clear();
    float penalityASMS = pow(dist_adj*fabs(state[0] - currentPredictRect[0])/((double)asms.lastPosition.width),2)  +
                         pow(dist_adj*fabs(state[1] - currentPredictRect[1])/((double)asms.lastPosition.height), 2);// +
                         //pow(dist_adj*fabs(state[2] - currentPredictRect[2])/(double)asms.lastPosition.width,2);
    float uncertainty = 1e-4*exp(-3.5*(conf_adj*confidenceASMS - penalityASMS));
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
}

void tASMS::update(){
    //asms.update();
}

void tASMS::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}

cv::Rect tASMS::getRect(){
    cv::Rect rect;
    rect.x = round(asms.lastPosition.x);
    rect.y = round(asms.lastPosition.y);
    rect.width = round(asms.lastPosition.width);
    rect.height = round(asms.lastPosition.height);
    return rect;
}
