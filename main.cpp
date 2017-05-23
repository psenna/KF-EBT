// VOT test

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "trackers/tasms.h"
#include "trackers/tkcf.h"
#include "trackers/tcbt.h"
#include "trackers/tmosse.h"
#include "trackers/tvdp.h"
#include "trackers/tgrayasms.h"
#include "trackers/tncc.h"
#include "kfebt.h"
#include "trax.h"


int main(int argc, char *argv[]){

    float ajuste = 0.30;

    // Alocate trackers
    tASMS asms(ajuste, 1.0);
    tKCF kcf(ajuste, 1.10);
    tCBT cbt(ajuste, 0.45);
    tVDP vdp(ajuste, 0.60);
    tncc ncc(ajuste, 0.75);
    //tGrayASMS gasms(ajuste, 0.7);

    KFEBT fusion;

    std::vector<BTracker*> trackers;
    //trackers.push_back(&cbt);
    //trackers.push_back(&vdp);
    trackers.push_back(&kcf);
    trackers.push_back(&asms);
    trackers.push_back(&ncc);
    //trackers.push_back(&gasms);

    trax_handle* trax;
    trax_metadata* config = trax_metadata_create(TRAX_REGION_RECTANGLE, TRAX_IMAGE_PATH, "KFebT", "KFebT", "none");
    trax_image* img = NULL;
    trax_region* rect = NULL;

    // Call trax_server_setup to initialize trax protocol
    trax = trax_server_setup(config, trax_no_log);

    cv::Rect region;
    cv::Mat image;
    std::vector<float> uncertainty, trackersResults;
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

            // Inicializatiuon
            for(unsigned int i = 0; i < trackers.size(); i++){
                trackers[i]->init(image, region);
            }

            // Alocate KFEBT
            fusion = KFEBT(9, 3*trackers.size(), 0, 0.05, region);
            if(argc >= 7){
                float adj = atof(argv[5]);
                fusion.setProcessCov(adj);
            }


            trax_server_reply(trax, rect, NULL);

        } else if (tr == TRAX_FRAME) {

            // Read image
            image = cv::imread(img->data);
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
            }

            // Correct the KF
            fusion.correct(trackersResults, uncertainty);

            // Model Update
            for(unsigned int i = 0; i < trackers.size(); i++){
                trackers[i]->start();
            }

            //Report result
            cv::Rect output = fusion.getResult();
            trax_region* region = trax_region_create_rectangle(output.x, output.y, output.width, output.height);
            trax_server_reply(trax, region, NULL);
            trax_region_release(&region);

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


