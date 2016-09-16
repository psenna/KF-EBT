#include "tasms.h"

tASMS::tASMS()
{

}

void tASMS::init(cv::Mat image, cv::Rect region){
    asms.init(image, region.x, region.y, region.x + region.width, region.y + region.height);
}

void tASMS::correctState(std::vector<float> st){
    this->state = st;
    asms.lastPosition.height *= st[2]/asms.lastPosition.width;
    asms.lastPosition.width = st[2];
    asms.lastPosition.x = st[0] - asms.lastPosition.width/2;
    asms.lastPosition.y = st[1] - asms.lastPosition.height/2;
}

void tASMS::track(cv::Mat image){
    double confidenceASMS = 0;
    cv::Rect asmsRect;
    BBox * bb = asms.track(image, &confidenceASMS);

    if (bb != NULL){
        asmsRect = cv::Rect(bb->x, bb->y, bb->width, bb->height);
        delete bb;
    }
}

void tASMS::run(){

}
