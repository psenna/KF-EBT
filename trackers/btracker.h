#ifndef BTRACKER_H
#define BTRACKER_H

/* Base tracker structure for KFEBT
 *
 */

#include <opencv2/opencv.hpp>
#include <QThread>
#include <vector>
#include <cmath>

#define DIST_ADJ 0.30

class BTracker : public QThread
{
    Q_OBJECT

public:
    bool ok;
    bool updateModel;
    float dist_adj;
    float conf_adj;
    double ratio;
    std::vector<float> state;
    std::vector<float> stateUncertainty;

    virtual void init(cv::Mat& image, cv::Rect region) = 0;
    virtual void correctState(std::vector<float> st) = 0;
    virtual void track() = 0;
    virtual void update() = 0;
    virtual void newFrame(cv::Mat& image, std::vector<float> predictRect) = 0;

    virtual cv::Rect getRect() = 0;

};

#endif // BTRACKER_H
