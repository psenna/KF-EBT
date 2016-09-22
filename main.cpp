// VOT test

#include <opencv2/opencv.hpp>
#include "trackers/tasms.h"
#include "trackers/tkcf.h"
#include "kfebt.h"

#define VOT_RECTANGLE
#include "vot.h"

#define CAMERA 0


int main(){

    // Açocate trackers
    tASMS asms;
    tKCF kcf;

    std::vector<BTracker*> trackers;
    trackers.push_back(&asms);
    trackers.push_back(&kcf);

    VOT vot;
    cv::Rect region, predictedRegion;
    region << vot.region();
    cv::Mat image = cv::imread(vot.frame());
    vot.report(region);

    for(int i = 0; i < trackers.size(); i++){
        trackers[i]->init(image, region);
    }

    // Alocate KFEBT
    KFEBT fusion(9, 3*trackers.size(), 0, 0.05, region);
    fusion.predict();

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
        uncertainty.clear();
        trackersResults.clear();

        // Track
        for(int i = 0; i < trackers.size(); i++){
            trackers[i]->track(image, fusion.getPrediction());
            uncertainty.insert(uncertainty.end(), trackers[i]->stateUncertainty.begin(), trackers[i]->stateUncertainty.end());
            trackersResults.insert(trackersResults.end(), trackers[i]->state.begin(), trackers[i]->state.end());
        }

        // Correct


        // Model Update


        // Feedback
        for(int i = 0; i < trackers.size(); i++){
            trackers[i]->correctState(fusion.getFusion());
        }

        //Report

        fusion.predict();
    }
    return 0;
}


