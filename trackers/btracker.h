#ifndef BTRACKER_H
#define BTRACKER_H

/* Base tracker structure for KFEBT
 *
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <pthread.h>

#define DIST_ADJ 0.30

class BTracker
{

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

    void run()
    {
        if(updateModel){
            update();
            updateModel = false;
        } else {
            track();
            updateModel = true;
        }
    }

    bool start() {
      return (pthread_create(&_thread, NULL, InternalThreadEntryFunc, this) == 0);
    }

    /** Will not return until the internal thread has exited. */
    void wait() {
       (void) pthread_join(_thread, NULL);
    }

private:
   static void * InternalThreadEntryFunc(void * This) {((BTracker *)This)->run(); return NULL;}

   pthread_t _thread;

};


#endif // BTRACKER_H
