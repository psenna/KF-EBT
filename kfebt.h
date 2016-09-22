#ifndef KFEBT_H
#define KFEBT_H

#include <opencv2/opencv.hpp>
#include <vector>

class KFEBT
{
public:
    KFEBT(int nStates, int nMeasurements, int nInputs, double dt, cv::Rect initialState);
    void predict();
    void correct(std::vector<float> measures, std::vector<float> Uncertainty);
    std::vector<float> getFusion();
    std::vector<float> getPrediction();

private:
    cv::KalmanFilter KF;
    cv::Mat estimated;
    cv::Mat corrected;
    cv::Mat KFMeasures;

};

#endif // KFEBT_H
