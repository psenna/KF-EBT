#ifndef KFEBT_H
#define KFEBT_H

#include <opencv2/opencv.hpp>
#include <vector>

class KFEBT
{
public:
    KFEBT(int nStates, int nMeasurements, int nInputs, double dt);
    void predict();
    void correct(std::vector<float> measures, std::vector<float> Uncertainty);
    std::vector<float> getState();

private:
    cv::KalmanFilter KF;

};

#endif // KFEBT_H
