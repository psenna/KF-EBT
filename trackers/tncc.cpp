#include "tncc.h"

tncc::tncc()
{

}

void tncc::init(cv::Mat& image, cv::Rect region){

}

void tncc::correctState(std::vector<float> st){

}

void tncc::track(){

}

void tncc::update(){
}

void tncc::run(){
    if(updateModel){
        update();
    } else {
        track();
    }
}

void tncc::newFrame(cv::Mat &image, std::vector<float> predictRect){
    currentFrame = image;
    currentPredictRect = predictRect;
}

cv::Rect tncc::getRect(){
    return cv::Rect();
}
