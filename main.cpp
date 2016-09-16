// VOT test

#include <opencv2/opencv.hpp>
#include "trackers/kcf/kcf.h"
#include "trackers/kcf/adjust.h"
#include "trackers/ASMS/colotracker.h"
#include <sys/timeb.h>

//#define VOT_RECTANGLE
#include "vot.h"

#define CAMERA 0

bool mousePress;
bool rectOK;
cv::Rect initRect;

void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt);

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


int main(){

    // interface
    mousePress = false;
    rectOK = false;

    Adjust adj;
    ColorTracker asms;
    KCF_Tracker kcf;

    cv::Rect region, predictedRegion;
    cv::Mat image;
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
        cv::waitKey(100);
    }

    cv::VideoWriter outputVideo;
    cv::Size S = image.size();
    int ex = static_cast<int>(cam.get(CV_CAP_PROP_FOURCC));
    outputVideo.open("output.mp4", CV_FOURCC('H', '2', '6', '4'), cam.get(CV_CAP_PROP_FPS), S, true);
    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1.1;
    int thickness = 2;
    int baseline=0;
    cv::Size textSize = cv::getTextSize("ASMS ", fontFace, fontScale, thickness, &baseline);
    cv::Point textOrg((image.cols - textSize.width), (image.rows - textSize.height)+2);
    cv::Point textOrg2((image.cols - textSize.width), (image.rows - (textSize.height*2)-5));
    cv::Point textOrg3((image.cols - textSize.width), (image.rows - (textSize.height*3)-12));
    baseline += thickness;


    std::cout << "foi\n";

    region = initRect;
    asms.init(image, region.x, region.y, region.x + region.width, region.y + region.height);

    cv::Mat grayImage;
    cv::Point2f motion;
    cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    cv::Mat kcfPatch = adj.init(grayImage, region);
    cv::imshow("Patch", kcfPatch);
    kcf.init(kcfPatch);

    cv::KalmanFilter KF;         // instantiate Kalman Filter
    int nStates = 9;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of action control
    double dt = 0.05;           // time between measurements (1/FPS)
    initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);    // init function
    cv::Mat KFMeasures = cv::Mat::zeros(nMeasurements,1, CV_64F);
    KF.statePre.at<double>(0) = region.x;
    KF.statePre.at<double>(0) = region.y;
    KF.statePre.at<double>(0) = region.width;
    KF.statePost.at<double>(0) = region.x;
    KF.statePost.at<double>(0) = region.y;
    KF.statePost.at<double>(0) = region.width;
    predictedRegion = region;
    cv::Mat estimated;

    estimated = KF.predict();

    while (true) {

        cam >> image;
        if(image.empty()){
            break;
        }

        double time_profile_counter = cv::getCPUTickCount();

        cv::cvtColor(image, grayImage, CV_BGR2GRAY);

        double confidenceASMS = 0;
        cv::Rect asmsRect;
        BBox * bb = asms.track(image, &confidenceASMS);

        if (bb != NULL){
            asmsRect = cv::Rect(bb->x, bb->y, bb->width, bb->height);
            delete bb;
        }

        kcfPatch = adj.getRect(grayImage, region);
        kcf.track(kcfPatch);
        motion = kcf.getError();
        motion.x *= adj.ratio[0];
        motion.y *= adj.ratio[1];

        //cv::imshow("1", kcfPatch);

        cv::Rect kcfRect = region;
        kcfRect.x += motion.x;
        kcfRect.y += motion.y;

        kcfPatch = adj.getRect(grayImage, kcfRect);
        kcf.updateKernel(kcfPatch);

        //cv::imshow("2", kcfPatch);

        double penalityASMS = 0;
        double penalityKCF = 0;


        // distance penality
        double adj = 0.25;
        penalityASMS = pow(adj*fabs(asmsRect.x - predictedRegion.x)/((double)asmsRect.width),2)  + pow(adj*fabs(asmsRect.y - predictedRegion.y)/((double)asmsRect.height), 2);
        penalityKCF =  pow(adj*fabs(kcfRect.x  - predictedRegion.x)/((double)kcfRect.width) ,2)  + pow(adj*fabs(kcfRect.y  - predictedRegion.y)/((double)kcfRect.height) , 2);

        //std::cout << "Confidence " << confidenceASMS << "  " << penalityASMS << "  " << kcf.correlation << "  " << penalityKCF <<  "\n";

        // ASMS confidence
        KF.measurementNoiseCov.at<double>(0,0) = 1e-4*exp(-3.5*(1.0*confidenceASMS - penalityASMS));
        KF.measurementNoiseCov.at<double>(1,1) = KF.measurementNoiseCov.at<double>(0,0);
        KF.measurementNoiseCov.at<double>(4,4) = KF.measurementNoiseCov.at<double>(0,0);

        // KCF confidence
        KF.measurementNoiseCov.at<double>(2,2) = 1e-4*exp(-3.5*(1.1*kcf.correlation - penalityKCF));
        KF.measurementNoiseCov.at<double>(3,3) = KF.measurementNoiseCov.at<double>(2,2);
        KF.measurementNoiseCov.at<double>(5,5) = 8*KF.measurementNoiseCov.at<double>(2,2);

        //std::cout << "Noise " << KF.measurementNoiseCov.at<double>(0,0) << " KCF  " <<  KF.measurementNoiseCov.at<double>(2,2) <<  "\n";

        KFMeasures.at<double>(0) = asms.lastPosition.x + asms.lastPosition.width/2;
        KFMeasures.at<double>(1) = asms.lastPosition.y + asms.lastPosition.height/2;
        KFMeasures.at<double>(2) = (float)region.x + motion.x + ((float)region.width)/2;
        KFMeasures.at<double>(3) = (float)region.y + motion.y + ((float)region.height)/2;
        KFMeasures.at<double>(4) = asms.lastPosition.width;
        KFMeasures.at<double>(5) = region.width;

        //std::cout << KF.statePost.at<double>(0) << "  " << KF.statePost.at<double>(1) << "  " << KF.statePost.at<double>(2) << "\n";
        //std::cout << KFMeasures.at<double>(0) << "  " << KFMeasures.at<double>(1) << "  " << KFMeasures.at<double>(2) << "  " << KFMeasures.at<double>(3) << "  " << KFMeasures.at<double>(4) << " " << KFMeasures.at<double>(5) <<  "\n";
        estimated = KF.correct(KFMeasures);
        //std::cout << "Measure: " << estimated.at<double>(0) << "  " << estimated.at<double>(1) << "  " << estimated.at<double>(2) << "  " << estimated.at<double>(3)  << "  " << estimated.at<double>(4) << "  " << estimated.at<double>(5) << "  " << estimated.at<double>(6)  << "\n \n";

        asms.lastPosition.height *= estimated.at<double>(2)/asms.lastPosition.width;
        asms.lastPosition.width = estimated.at<double>(2);
        asms.lastPosition.x = estimated.at<double>(0) - asms.lastPosition.width/2;
        asms.lastPosition.y = estimated.at<double>(1) - asms.lastPosition.height/2;


        region.x = asms.lastPosition.x;
        region.y = asms.lastPosition.y;
        region.width = asms.lastPosition.width;
        region.height = asms.lastPosition.height;


        time_profile_counter = cv::getCPUTickCount() - time_profile_counter;
        double time = time_profile_counter/((double)cvGetTickFrequency()*1000);
        double fps = 1000.0/time;
        //std::cout << "FPS: " << fps << "  Time: " << time << " ms\n";

        cv::Mat result = image.clone();
        cv::rectangle(result, kcfRect, cv::Scalar(255, 0, 0));
        cv::rectangle(result, asmsRect, cv::Scalar(0,0,255));
        cv::rectangle(result, region, cv::Scalar(0, 255, 0),2);
        putText(result, "ASMS ", textOrg,  fontFace, fontScale, cv::Scalar(0,0,255), thickness, 8);
        putText(result, "KCF  ", textOrg2, fontFace, fontScale, cv::Scalar(255, 0, 0), thickness, 8);
        putText(result, "FUSED", textOrg3, fontFace, fontScale, cv::Scalar(0, 255, 0), thickness, 8);
        outputVideo << result;
        cv::imshow("result", result);

        estimated = KF.predict();
        //std::cout << "Predict: " <<  estimated.at<double>(0) << "  " << estimated.at<double>(1) << "  " << estimated.at<double>(2) << "  " << estimated.at<double>(3) << "  " << estimated.at<double>(4) << "  " << estimated.at<double>(5) << "  " << estimated.at<double>(6)  << "\n";


        predictedRegion.height *= estimated.at<double>(2)/asms.lastPosition.width;
        predictedRegion.width = estimated.at<double>(2);
        predictedRegion.x = estimated.at<double>(0) - (estimated.at<double>(2))/2;
        predictedRegion.y = estimated.at<double>(1) - (estimated.at<double>(2)/asms.lastPosition.width)/2;

        //cv::imshow("Final", adj.getRect(image, region));


        char key = cv::waitKey(1);
        if(key == 27)
            break;
    }
    outputVideo.release();
    //cv::waitKey(0);
    return 0;
}

void initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt)
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
    //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
    //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]

    KF.measurementMatrix.at<double>(0,0) = 1;  // x ASMS
    KF.measurementMatrix.at<double>(1,1) = 1;  // y ASMS
    KF.measurementMatrix.at<double>(2,0) = 1;  // x KCF
    KF.measurementMatrix.at<double>(3,1) = 1;  // y KCF
    KF.measurementMatrix.at<double>(4,2) = 1; // Scale ASMS
    KF.measurementMatrix.at<double>(5,2) = 1;
}

