// VOT test

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "trackers/tasms.h"
#include "trackers/tkcf.h"
#include "trackers/tcbt.h"
#include "kfebt.h"
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/kalman_filters.hpp>

#define VOT_RECTANGLE
#include "vot.h"

#define CAMERA 0


int main(){

    // Alocate trackers
    tASMS asms;
    tKCF kcf;
    tCBT cbt;

    std::vector<BTracker*> trackers;
    trackers.push_back(&cbt);
    trackers.push_back(&kcf);
    trackers.push_back(&asms);

    VOT vot;
    cv::Rect region;
    region << vot.region();
    cv::Mat image = cv::imread(vot.frame());
    vot.report(region);

    for(unsigned int i = 0; i < trackers.size(); i++){
        trackers[i]->init(image, region);
    }

    // Alocate KFEBT
    KFEBT fusion(9, 3*trackers.size(), 0, 0.05, region);

    std::vector<float> uncertainty, trackersResults;

    while (!vot.end()){
        string imagepath = vot.frame();
        //std::cout << imagepath << "\n";
        if (imagepath.empty()){
            break;
        }

        image = cv::imread(imagepath);
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
            trackers[i]->updateModel = true;
        }

        // Correct the KF
        fusion.correct(trackersResults, uncertainty);

        // Model Update
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->start();
        }

        //Report
        vot.report(fusion.getResult());

        // Wait trackers update process
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->wait();
            trackers[i]->updateModel = false;
        }

        // Feedback
        for(unsigned int i = 0; i < trackers.size(); i++){
            trackers[i]->correctState(fusion.getFusion());
        }


        /*cv::rectangle(image, fusion.getResult(), cv::Scalar(255, 0, 0));
        cv::imshow("resp", image);
        cv::waitKey(0);*/

        fusion.predict();

    }
    return 0;
}


