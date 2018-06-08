
#include <opencv2/opencv.hpp>
#include "kfebtracker.h"

#define CAMERA 0

bool mousePress;
bool rectOK;
cv::Rect initRect;

static void onMouse( int event, int x, int y, int, void* ){
    if( event != cv::EVENT_LBUTTONDOWN || rectOK)
        return;

    if(mousePress){
        initRect.width = x - initRect.x;
        initRect.height = y - initRect.y;
        rectOK = true;
    }

    if(!mousePress){
        initRect.x = x;
        initRect.y = y;
        mousePress = true;
    }
}


int main(void){

    // interface
    mousePress = false;
    rectOK = false;

    KFebTracker tracker;
    tracker.init("AK");

    cv::Mat image;
    bool run = 1;

    cv::VideoCapture cam(CAMERA);
    if(!cam.isOpened()){
        std::cout << "Error: Camera not found!\n";
        return 1;
    }
    cam >> image;
    cv::imshow("result", image);
    cv::setMouseCallback("result", onMouse,0);

    while(!mousePress){
        cam >> image;
        cv::imshow("result", image);
        cv::waitKey(10);
    }

    while(!rectOK){
        cv::waitKey(10);
    }

    tracker.initTrackers(image, initRect);

    while (run){

        // Read image
        cam >> image;
        if(image.empty()){
            break;
        }

        //Report result
        cv::Rect output = tracker.track(image);
        cv::Mat saida = image.clone();
        cv::rectangle(saida, output, cv::Scalar(0,255,0), 2);
        cv::imshow("result", saida);

        char key = cv::waitKey(10);
        if(key == 27)
            break;

    }
    return 0;
}

