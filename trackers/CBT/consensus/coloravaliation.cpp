#include "coloravaliation.h"

ColorAvaliation::ColorAvaliation()
{

}


void ColorAvaliation::init(cv::Mat &image){
    model = cv::Mat::zeros(1,BINS*BINS*BINS, CV_32FC1);

    int ch = image.channels();
    int cols = image.cols;
    uint8_t* pixelPtr = (uint8_t*)image.data;
    int id[3];
    int id1;
    float total = 0;

    int rangePerBin = 256/BINS;
    float rangePerBinInv = 1./(float)rangePerBin;


    for(int i = 0; i < image.rows; i++){
        for(int j = 0; j < image.cols; j++){
            id1 = i*cols + j*ch;
            id[0] = pixelPtr[id1]*rangePerBinInv;
            id[1] = pixelPtr[++id1]*rangePerBinInv;
            id[2] = pixelPtr[++id1]*rangePerBinInv;

            id1 = id[0]*BINS*BINS + id[1]*BINS + id[2];

            model.at<float>(0, id1)++;
            total++;
        }
    }

    for(int i = 0; i < model.cols; i++){
        model.at<float>(0, i)/=total;
    }
}

void ColorAvaliation::update(cv::Mat &image){
    histogran = cv::Mat::zeros(1,BINS*BINS*BINS, CV_32FC1);

    int ch = image.channels();
    int cols = image.cols;
    uint8_t* pixelPtr = (uint8_t*)image.data;
    int id[3];
    int id1;
    float total = 0;

    int rangePerBin = 256/BINS;
    float rangePerBinInv = 1./(float)rangePerBin;


    for(int i = 0; i < image.rows; i++){
        for(int j = 0; j < image.cols; j++){
            id1 = i*cols + j*ch;
            id[0] = pixelPtr[id1]*rangePerBinInv;
            id[1] = pixelPtr[++id1]*rangePerBinInv;
            id[2] = pixelPtr[++id1]*rangePerBinInv;

            id1 = id[0]*BINS*BINS + id[1]*BINS + id[2];

            histogran.at<float>(0, id1)++;
            total++;
        }
    }

    for(int i = 0; i < histogran.cols; i++){
        histogran.at<float>(0, i)/=total;
    }

    model = model*(1-ADAPTATION) + histogran*ADAPTATION;
}

float ColorAvaliation::compare(cv::Mat &image){
    histogran = cv::Mat::zeros(1,BINS*BINS*BINS, CV_32FC1);

    int ch = image.channels();
    int cols = image.cols;
    uint8_t* pixelPtr = (uint8_t*)image.data;
    int id[3];
    int id1;
    float total = 0;

    int rangePerBin = 256/BINS;
    float rangePerBinInv = 1./(float)rangePerBin;


    for(int i = 0; i < image.rows; i++){
        for(int j = 0; j < image.cols; j++){
            id1 = i*cols + j*ch;
            id[0] = pixelPtr[id1]*rangePerBinInv;
            id[1] = pixelPtr[++id1]*rangePerBinInv;
            id[2] = pixelPtr[++id1]*rangePerBinInv;

            id1 = id[0]*BINS*BINS + id[1]*BINS + id[2];

            histogran.at<float>(0, id1)++;
            total++;
        }
    }

    for(int i = 0; i < histogran.cols; i++){
        histogran.at<float>(0, i)/=total;
    }

    double similarity = 0;

    for (int i=0; i < histogran.cols; i++) {
        similarity += sqrt(histogran.at<float>(0, i)*model.at<float>(0, i));
    }
    return similarity;
}
