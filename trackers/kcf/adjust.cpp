#include "adjust.h"

Adjust::Adjust()
{

}

cv::Mat Adjust::init(cv::Mat& image, cv::Rect rect){

    float h = rect.height;
    float w = rect.width;

    float border_h = h;
    float border_w = w;

    if(h < w){
        if(h < MINSIZE){
            w *= (MINSIZE/h);
            h = MINSIZE;
        }
    } else {
        if(w < MINSIZE){
            h *= (MINSIZE/w);
            w = MINSIZE;
        }
    }
    h *= KCF_PADDING;
    w *= KCF_PADDING;

    h = cv::getOptimalDFTSize(h);
    w = cv::getOptimalDFTSize(w);

    size.width = w;
    size.height = h;

    border_h = (h-border_h)/2;
    border_w = (w-border_w)/2;

    srcTri[0] = cv::Point2f(rect.x, rect.y);
    srcTri[1] = cv::Point2f(rect.x, rect.y+rect.height);
    srcTri[2] = cv::Point2f(rect.x+rect.width, rect.y+rect.height);

    dstObj[0] = cv::Point2f(border_w, border_h);
    dstObj[1] = cv::Point2f(border_w, size.height-border_h);
    dstObj[2] = cv::Point2f(size.width-border_w, size.height-border_h);

    cv::Mat transformMatrix = cv::getAffineTransform(srcTri, dstObj);
    cv::Mat result;

    cv::warpAffine(image, result, transformMatrix, this->size, cv::INTER_LINEAR);

    ratio[0] = cv::norm(srcTri[1]-srcTri[2])/cv::norm(dstObj[1]-dstObj[2]);
    ratio[1] = cv::norm(srcTri[0]-srcTri[1])/cv::norm(dstObj[0]-dstObj[1]);
    return result;

}

cv::Mat Adjust::getRect(cv::Mat& image, cv::Rect rect){
    srcTri[0] = cv::Point2f(rect.x, rect.y);
    srcTri[1] = cv::Point2f(rect.x, rect.y+rect.height);
    srcTri[2] = cv::Point2f(rect.x+rect.width, rect.y+rect.height);

    cv::Mat transformMatrix = cv::getAffineTransform(srcTri, dstObj);
    cv::Mat result;

    cv::warpAffine(image, result, transformMatrix, this->size, cv::INTER_LINEAR);

    ratio[0] = cv::norm(srcTri[0]-srcTri[1])/cv::norm(dstObj[0]-dstObj[1]);
    ratio[1] = cv::norm(srcTri[1]-srcTri[2])/cv::norm(dstObj[1]-dstObj[2]);

    return result;
}
