// VOT test

#include <opencv2/opencv.hpp>
#include <pthread.h>
#include "kfebtracker.h"
#include "trax.h"


int main(void){

    KFebTracker tracker;
    tracker.init("AKN");

    trax_handle* trax;
    trax_metadata* config = trax_metadata_create(TRAX_REGION_RECTANGLE, TRAX_IMAGE_PATH, "KFebT", "KFebT", "none");
    trax_image* img = NULL;
    trax_region* rect = NULL;

    // Call trax_server_setup to initialize trax protocol
    trax = trax_server_setup(config, trax_no_log);

    cv::Rect region;
    cv::Mat image;

    bool run = 1;

    while (run){

        trax_properties* prop = trax_properties_create();
        int tr = trax_server_wait(trax, &img, &rect, prop);


        if (tr == TRAX_INITIALIZE){

            float x, y, width, height;
            trax_region_get_rectangle(rect, &x, &y, &width, &height);
            region.x = x;
            region.y = y;
            region.width = width;
            region.height = height;
            image = cv::imread(img->data);

            // Initialize trackers
            tracker.initTrackers(image, region);

            trax_server_reply(trax, rect, NULL);

        } else if (tr == TRAX_FRAME) {

            // Read image
            image = cv::imread(img->data);
            if(image.empty()){
                break;
            }

            //Report result
            cv::Rect output = tracker.track(image);
            trax_region* region = trax_region_create_rectangle(output.x, output.y, output.width, output.height);
            trax_server_reply(trax, region, NULL);
            trax_region_release(&region);

        } else {
            run = 0;
        }

        trax_properties_release(&prop);
        trax_metadata_release(&config);

        /*cv::rectangle(image, fusion.getResult(), cv::Scalar(255, 0, 0));
        cv::imshow("resp", image);
        cv::waitKey(0);*/

    }
    return 0;
}


