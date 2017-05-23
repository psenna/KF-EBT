#include "tncc.h"

tncc::tncc(float dist_adj, float conf_adj)
{
    this->dist_adj = dist_adj;
    this->conf_adj = conf_adj;
}

void tncc::init(cv::Mat& image, cv::Rect region){
    ncc.init(image, region);
    ratio = (float)region.height/(float)region.width;
    updateModel = false;
}

void tncc::correctState(std::vector<float> st){
    this->state = st;
    ncc.p_position = cv::Point2f(st[0], st[1]);
    ncc.p_size.width = st[2];
    ncc.p_size.height = st[2]*ratio;
}

void tncc::track(){
    double confidence =  ncc.track(currentFrame);

    // write state
    state.clear();
    state.push_back(ncc.p_position.x);
    state.push_back(ncc.p_position.y);
    state.push_back(ncc.p_size.width);

    this->stateUncertainty.clear();
    float penalityNCC = pow(dist_adj*fabs(state[0] - currentPredictRect[0])/((double)ncc.p_size.width),2)  +
                         pow(dist_adj*fabs(state[1] - currentPredictRect[1])/((double)ncc.p_size.height), 2);
    float uncertainty = 1e-4*exp(-3.5*(conf_adj*confidence - penalityNCC));
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(5*uncertainty);

}

void tncc::update(){
    // None
}


void tncc::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}

cv::Rect tncc::getRect(){
    // ToDo
    return cv::Rect();
}
