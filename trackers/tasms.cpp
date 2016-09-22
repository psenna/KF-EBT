#include "tasms.h"

tASMS::tASMS()
{

}

void tASMS::init(cv::Mat& image, cv::Rect region){
    asms.init(image, region.x, region.y, region.x + region.width, region.y + region.height);
    ratio = (float)region.height/(float)region.width;
}

void tASMS::correctState(std::vector<float> st){
    this->state = st;
    asms.lastPosition.height = st[2]*ratio;
    asms.lastPosition.width = st[2];
    asms.lastPosition.x = st[0] - asms.lastPosition.width/2;
    asms.lastPosition.y = st[1] - asms.lastPosition.height/2;
}

void tASMS::track(cv::Mat& image, std::vector<float> predictRect){
    double confidenceASMS = 0;
    cv::Rect asmsRect;
    BBox * bb = asms.track(image, &confidenceASMS);

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
    float penalityASMS = pow(DIST_ADJ*fabs(state[0] - predictRect[0])/((double)asms.lastPosition.width),2)  + pow(DIST_ADJ*fabs(state[1] - predictRect[1])/((double)asms.lastPosition.height), 2);
    float uncertainty = 1e-4*exp(-3.5*(1.0*confidenceASMS - penalityASMS));
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
}

void tASMS::update(cv::Mat& image){
    asms.update();
}

void tASMS::run(){

}
