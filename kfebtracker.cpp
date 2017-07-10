#include "kfebtracker.h"

KFebTracker::KFebTracker()
{
    asms = tASMS(ajuste, 0.90);
    kcf = tKCF(ajuste, 1.15);
    cbt = tCBT(ajuste, 0.45);
    vdp = tVDP(ajuste, 0.60);
    ncc = tncc(ajuste, 0.7);
}

/* Initiation Parameters
 * A = ASMS
 * K = KCF
 * C = CBT
 * V = VDP
 * N = NCC
 *
 * (Send string "AKN" to initialize the tracker with ASMS, KCF and NCC)
 */
void KFebTracker::init(std::string initPar){
    trackers.clear();
    for(int i = 0; i < (int)initPar.length(); i++){
        switch (initPar[i]) {
        case 'A':
            trackers.push_back(&asms);
            break;
        case 'K':
            trackers.push_back(&kcf);
            break;
        case 'C':
            trackers.push_back(&cbt);
            break;
        case 'V':
            trackers.push_back(&vdp);
            break;
        case 'N':
            trackers.push_back(&ncc);
            break;
        default:
            break;
        }
    }
}

void KFebTracker::initTrackers(cv::Mat image, cv::Rect region){
    // Inicialization
    for(unsigned int i = 0; i < trackers.size(); i++){
        trackers[i]->init(image, region);
    }

    // Alocate KFEBT
    fusion = KFEBT(9, 3*trackers.size(), 0, 0.05, region);
}

cv::Rect KFebTracker::track(cv::Mat image){
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

    //Report result
    return fusion.getResult();
}
