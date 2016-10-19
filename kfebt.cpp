#include "kfebt.h"


KFEBT::KFEBT(){

}

KFEBT::KFEBT(int nStates, int nMeasurements, int nInputs, double dt, cv::Rect initialState)
{
    KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
    cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(0.8*1e-4));       // set process noise
    cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-5));   // set measurement noise
    cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
    /* DYNAMIC MODEL */
    //  [1 0 d dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
    //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]

    // position
    KF.transitionMatrix.at<double>(0,3) = dt;
    KF.transitionMatrix.at<double>(1,4) = dt;
    KF.transitionMatrix.at<double>(2,5) = dt;
    KF.transitionMatrix.at<double>(3,6) = dt;
    KF.transitionMatrix.at<double>(4,7) = dt;
    KF.transitionMatrix.at<double>(5,8) = dt;
    KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
    KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);

    /* MEASUREMENT MODEL */
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    for(int i = 0; i < nMeasurements/3; i++){
        KF.measurementMatrix.at<double>(i*3,0) = 1;    // x for Tracker i
        KF.measurementMatrix.at<double>(i*3+1,1) = 1;  // y for Tracker i
        KF.measurementMatrix.at<double>(i*3+2,2) = 1;  // Scale for Tracker i
    }

    KFMeasures = cv::Mat::zeros(nMeasurements,1, CV_64F);

    KF.statePre.at<double>(0) = (double)initialState.x + (double)(initialState.width)/2.0;
    KF.statePre.at<double>(1) = (double)initialState.y + (double)(initialState.height)/2.0;
    KF.statePre.at<double>(2) = (double)initialState.width;
    KF.statePost.at<double>(0) = (double)initialState.x + (double)(initialState.width)/2.0;
    KF.statePost.at<double>(1) = (double)initialState.y + (double)(initialState.height)/2.0;
    KF.statePost.at<double>(2) = (double)initialState.width;

    estimated = KF.predict();
    corrected = estimated.clone();
    ratio = (float)initialState.height/(float)initialState.width;
}


void KFEBT::predict(){
    estimated = KF.predict();
}

void KFEBT::correct(std::vector<float> measures, std::vector<float> Uncertainty){
    for(int i = 0; i < measures.size(); i++){
        KFMeasures.at<double>(i) = measures[i];
        KF.measurementNoiseCov.at<double>(i,i) = Uncertainty[i];
    }
    corrected = KF.correct(KFMeasures);
}

std::vector<float> KFEBT::getFusion(){
    std::vector<float> ret;
    ret.push_back(corrected.at<double>(0));
    ret.push_back(corrected.at<double>(1));
    ret.push_back(corrected.at<double>(2));
    return ret;
}

cv::Rect KFEBT::getResult(){
    cv::Rect result;
    result.height = round(corrected.at<double>(2)*ratio);
    result.width = round(corrected.at<double>(2));
    result.x = round(corrected.at<double>(0) - corrected.at<double>(2)/2.0);
    result.y = round(corrected.at<double>(1) - (corrected.at<double>(2)*ratio/2.0));
    return result;
}

std::vector<float> KFEBT::getPrediction(){
    std::vector<float> ret;
    ret.push_back(estimated.at<double>(0));
    ret.push_back(estimated.at<double>(1));
    ret.push_back(estimated.at<double>(2));
    return ret;
}

