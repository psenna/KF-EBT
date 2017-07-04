#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "trackers/tasms.h"
#include "trackers/tkcf.h"
#include "trackers/tcbt.h"
#include "trackers/tmosse.h"
#include "trackers/tvdp.h"
#include "trackers/tncc.h"
#include "kfebt.h"
#include "trax.h"

#define CAMERA 0

bool mousePress;
bool rectOK;
cv::Rect initRect;

static void onMouse( int event, int x, int y, int, void* ){
    if( event != cv::EVENT_LBUTTONDOWN || rectOK)
        return;

    std::cout << "event " << (int) mousePress << " " << (int) rectOK << "\n";

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


int main(int argc, char *argv[]){

    // interface
    mousePress = false;
    rectOK = false;

    float ajuste = 0.15;
    // Alocate trackers

    tASMS asms(ajuste, 0.90);
    tKCF kcf(ajuste, 1.15);
    tCBT cbt(ajuste, 0.45);
    tVDP vdp(ajuste, 0.60);
    tncc ncc(ajuste, 0.7);


    KFEBT fusion;

    std::vector<BTracker*> trackers;
    //trackers.push_back(&cbt);
    //trackers.push_back(&vdp);
    trackers.push_back(&kcf);
    trackers.push_back(&asms);
    trackers.push_back(&ncc);

    cv::Rect region;
    cv::Mat image;
    std::vector<float> uncertainty, trackersResults;
    bool run = 1;

    cv::VideoCapture cam(CAMERA);
    cam >> image;
    cv::imshow("result", image);
    cv::setMouseCallback("result", onMouse,0);
    std::cout << image.size() << "\n";

    while(!mousePress){
        cam >> image;
        cv::imshow("result", image);
        cv::waitKey(10);
    }

    while(!rectOK){
        cv::waitKey(10);
    }

    // Init trackers
    region = initRect;
    for(unsigned int i = 0; i < trackers.size(); i++){
        trackers[i]->init(image, initRect);
    }

    // Alocate KFEBT
    fusion = KFEBT(9, 3*trackers.size(), 0, 0.05, initRect);
    if(argc >= 7){
        float adj = atof(argv[5]);
        fusion.setProcessCov(adj);
    }

    while (run){

        // Read image
        cam >> image;
        if(image.empty()){
            break;
        }

        // Start trackers
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->newFrame(image, fusion.getPrediction());
            trackers[i]->start();
        }

        // Wait and get results
        uncertainty.clear();
        trackersResults.clear();
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->wait();
            uncertainty.insert(uncertainty.end(), trackers[i]->stateUncertainty.begin(), trackers[i]->stateUncertainty.end());
            trackersResults.insert(trackersResults.end(), trackers[i]->state.begin(), trackers[i]->state.end());
        }

        // Correct the KF
        fusion.correct(trackersResults, uncertainty);

        // Model Update
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->start();
        }

        //Report result
        cv::Rect output = fusion.getResult();
        cv::Mat saida = image.clone();
        cv::rectangle(saida, output, cv::Scalar(0,255,0), 2);
        cv::imshow("result", saida);

        // Wait trackers update process
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->wait();
        }

        // Feedback
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->correctState(fusion.getFusion());
        }

        // Predict next frame state
        fusion.predict();

        char key = cv::waitKey(10);
        if(key == 27)
            break;

    }
    return 0;
}

