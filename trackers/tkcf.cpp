#include "tkcf.h"

tKCF::tKCF(float dist_adj, float conf_adj)
{
    this->dist_adj = dist_adj;
    this->conf_adj = conf_adj;
}

void tKCF::init(cv::Mat& image, cv::Rect region){
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    cv::Mat kcfPatch = adj.init(grayImage, region);
    kcf.init(kcfPatch);
    ratio = (float)region.height/(float)region.width;
    this->region = region;
    updateModel = false;
    state.push_back(region.x);
    state.push_back(region.y);
    state.push_back(region.width);
}

void tKCF::correctState(std::vector<float> st){
    this->state = st;
    this->region.height = round(st[2]*ratio);
    this->region.width = round(st[2]);
    this->region.x = round(st[0] - st[2]/2.0);
    this->region.y = round(st[1] - (st[2]*ratio/2.0));
}

void tKCF::track(){
    /*cv::Mat imag = currentFrame.clone();
    cv::rectangle(imag, region, cv::Scalar(0, 255, 0));*/

    cv::Point2f motion;
    cv::cvtColor(currentFrame, grayImage, CV_BGR2GRAY);
    cv::Mat kcfPatch = adj.getRect(grayImage, region);
    kcf.track(kcfPatch);
    motion = kcf.getError();
    motion.x *= adj.ratio[0];
    motion.y *= adj.ratio[1];
    float width = state[2];

    state.clear();
    state.push_back(round((float)region.x + motion.x + (float)region.width/2.0));
    state.push_back(round((float)region.y + motion.y + (float)region.height/2.0));
    state.push_back(width);

    stateUncertainty.clear();
    float penalityKCF = pow(dist_adj*fabs(state[0] - currentPredictRect[0])/((double)region.width),2)  +
                        pow(dist_adj*fabs(state[1] - currentPredictRect[1])/((double)region.height), 2);// +
                        //pow(dist_adj*fabs(state[2] - currentPredictRect[2])/((double)region.width),2);
    float uncertainty = 1e-4*exp(-3.5*(conf_adj*kcf.correlation - penalityKCF));
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty);
    stateUncertainty.push_back(uncertainty*5.0);

    region.x += round(motion.x);
    region.y += round(motion.y);

    /*cv::rectangle(imag, region, cv::Scalar(255, 0, 0));
    cv::imshow("aaaa", imag);*/
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

cv::Rect tKCF::getRect(){
    cv::Rect rect;
    rect.x = round(region.x);
    rect.y = round(region.y);
    rect.width = round(region.width);
    rect.height = round(region.height);
    return rect;
}
