#include "mosse.h"

Mosse::Mosse()
{

}

void Mosse::init(cv::Mat frame, cv::Rect rect){
    int x,y, w, h;
    w = cv::getOptimalDFTSize(rect.width);
    h = cv::getOptimalDFTSize(rect.height);
    x = rect.x + (int)(w/2);
    y = rect.y + (int)(h/2);

    this->position.x = x;
    this->position.y = y;
    this->size.width = w;
    this->size.height = h;

    cv::Mat img;
    cv::getRectSubPix(frame, cv::Size(w, h), this->position, img);

    cv::createHanningWindow(this->win, cv::Size(w, h), CV_32F);
    cv::Mat g = cv::Mat::zeros(h, w, CV_32FC1);
    g.at<float>((int)(h/2), (int)(w/2)) = 1;
    cv::GaussianBlur(g, g, cv::Size(-1,-1), 1.8);
    double max;
    cv::minMaxIdx(g, NULL, &max);
    g /= max;

    cv::dft(g, this->G, cv::DFT_COMPLEX_OUTPUT);

    this->H1 = cv::Mat::zeros(this->G.rows, this->G.cols, CV_32FC2);
    this->H2 = cv::Mat::zeros(this->G.rows, this->G.cols, CV_32FC2);

    //std::cout << H1.size() << "  " << G.size() << "  " << img.size() << "\n";

    std::srand(std::time(NULL));

    for(int i = 0; i < 128; i++){
        cv::Mat A, aux;
        this->rndWarp(img, A);
        this->preProcess(A, aux);
        cv::dft(aux, A, cv::DFT_COMPLEX_OUTPUT);
        cv::mulSpectrums(this->G, A, aux, 0, true);
        this->H1 += aux;
        cv::mulSpectrums(      A, A, aux, 0, true);
        this->H2 += aux;
    }
    this->updateKernel();
    this->update(frame);
}

void Mosse::preProcess(cv::Mat input, cv::Mat& output){
    cv::Mat aux;
    input.convertTo(aux, CV_32F);
    cv::log(aux+1, output);
    cv::Scalar mean, std;
    cv::meanStdDev(output, mean, std);
    output = (output-((mean.val[0])/(std.val[0]+EPS)));
    cv::multiply(output, this->win, output);
}

void Mosse::updateKernel(){
    divSpec(this->H1, this->H2, this->H);
    //std::vector<cv::Mat> channels;
    //cv::split(this->H, channels);
    //cv::multiply(channels[0], -1, channels[0]);
    //cv::merge(channels, this->H);
}

void Mosse::divSpec(cv::Mat A, cv::Mat B, cv::Mat& C){
    std::vector<cv::Mat> channelsA;
    std::vector<cv::Mat> channelsB;
    std::vector<cv::Mat> channelsC;
    cv::split(A, channelsA);
    cv::split(B, channelsB);
    cv::split(B, channelsC);

    cv::Mat aux, aux2, aux3, conj;
    cv::multiply(channelsB[0], channelsB[0], aux);
    cv::multiply(channelsB[1], channelsB[1], aux2);
    cv::add(aux, aux2, conj);

    //  (a*c) + (b*d)
    cv::multiply(channelsA[0], channelsB[0], aux);
    cv::multiply(channelsA[1], channelsB[1], aux2);
    cv::add(aux, aux2, aux3);
    cv::divide(aux3, conj, channelsC[0]);

    // (a*d) - (b*c)
    cv::multiply(channelsA[0], channelsB[1], aux);
    cv::multiply(channelsA[1], channelsB[0], aux2);
    cv::subtract(aux, aux2, aux3);
    cv::divide(aux3, conj, channelsC[1]);

    cv::merge(channelsC, C);
}

void Mosse::update(cv::Mat frame){
    cv::Mat image;
    cv::getRectSubPix(frame, this->size, this->position, image);
    this->lastImage = image;
    preProcess(image, image);

    cv::Mat resp;
    float psr;
    double max;
    cv::Point2f pos;
    correlate(image, resp, pos, psr, max);

    if(psr > 8.0){
        this->position += pos;
        cv::getRectSubPix(frame, this->size, this->position, image);
        this->lastImage = image;
        //cv::imshow("image", image);
        preProcess(image, image);

        cv::Mat faux;
        cv::cvtColor(frame, faux, CV_GRAY2BGR);
        cv::rectangle(faux, cv::Point2f(position.x-(size.width/2), position.y-(size.height/2)), cv::Point2f(position.x+(size.width/2), position.y+(size.height/2)), cv::Scalar(255, 0, 0));
        //cv::imshow("fram", faux);
        //cv::waitKey(0);

        cv::Mat A;
        cv::dft(image, A, cv::DFT_COMPLEX_OUTPUT);
        cv::Mat h1, h2;
        cv::mulSpectrums(this->G, A, h1, 0, true);
        cv::mulSpectrums(      A, A, h2, 0, true);
        cv::Mat aux1, aux2;
        cv::multiply(this->H1, (1.0-RATE), aux1);
        cv::multiply(      h1,     (RATE), aux2);
        cv::add(aux1, aux2, this->H1);
        cv::multiply(this->H2, (1.0-RATE), aux1);
        cv::multiply(      h2,     (RATE), aux2);
        cv::add(aux1, aux2, this->H2);
        this->updateKernel();
    }
}

cv::Point2f Mosse::getPosition(){
    return position;
}

cv::Rect Mosse::getRect(){
    cv::Rect r;
    r.x = position.x-(size.width/2);
    r.y = position.y-(size.height/2);
    r.width = size.width;
    r.height = size.height;
    return r;
}

void Mosse::setPosition(cv::Rect){

}

void Mosse::correlate(cv::Mat image, cv::Mat& resp, cv::Point2f& position, float& psr, double& max){
    cv::Mat dftImg, C;
    cv::dft(image, dftImg, cv::DFT_COMPLEX_OUTPUT);
    cv::mulSpectrums( dftImg, this->H, C, 0, true);
    cv::idft(C, resp, cv::DFT_SCALE | cv::DFT_REAL_OUTPUT);

    cv::Size s = resp.size();
    double min;
    cv::Point pMin, pMax;
    cv::minMaxLoc(resp, &min, &max, &pMin, &pMax, cv::Mat());
    position = pMax;
    cv::Mat sideResp = resp.clone();
    cv::rectangle(sideResp, cv::Point2f(position.x-5, position.y-5), cv::Point2f(position.x+5, position.y+5), 0, -1);
    cv::Scalar mean, std;
    cv::meanStdDev(sideResp, mean, std);
    psr = ((max-mean.val[0])/(std.val[0]+EPS));
    position.x = position.x+1-((float)s.width/2.0);
    position.y = position.y+1-((float)s.height/2.0);

    //cv::Mat Resp;
    //cv::normalize(resp, Resp, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
    //cv::imshow("respMosse", Resp);
    //cv::waitKey(0);
    //std::cout << max << "  " << position  << "  " << position.y << "  " << psr << "\n";
}

void Mosse::rndWarp(cv::Mat input, cv::Mat& output){
    float h = input.rows/2;
    float w = input.cols/2;

    cv::Mat T = cv::Mat::zeros(2,3,CV_32FC1);
    float coef = 0.1;
    float ang = (((double) std::rand() / (RAND_MAX)) - 0.5)*coef;

    float c = cos(ang);
    float s = sin(ang);
    T.at<float>(0,0) =  c + (((double) std::rand() / (RAND_MAX)) - 0.5)*coef;
    T.at<float>(0,1) = -s + (((double) std::rand() / (RAND_MAX)) - 0.5)*coef;
    T.at<float>(1,0) =  s + (((double) std::rand() / (RAND_MAX)) - 0.5)*coef;
    T.at<float>(1,1) =  c + (((double) std::rand() / (RAND_MAX)) - 0.5)*coef;


    T.at<float>(2,0) = w + (T.at<float>(0,0)*w + T.at<float>(1,0) *h);
    T.at<float>(2,1) = h + (T.at<float>(0,1)*w + T.at<float>(1,1) *h);

    output = cv::Mat::zeros(input.rows, input.cols, input.type());
    cv::warpAffine(input, output, T, output.size(), cv::INTER_CUBIC, cv::BORDER_REFLECT);
}

cv::Mat Mosse::conjug(cv::Mat M){
    std::vector<cv::Mat> channels;
    cv::split(M, channels);
    channels[1] *= -1;
    cv::Mat resp;
    cv::merge(channels, resp);
    return resp;
}

// To be used with Flow
void Mosse::init(cv::Mat target){

    this->position.x = (int) (target.cols/2);
    this->position.y = (int) (target.rows/2);
    this->size = target.size();

    cv::createHanningWindow(this->win, this->size, CV_32F);
    cv::Mat g = cv::Mat::zeros(this->size.height, this->size.width, CV_32FC1);
    g.at<float>(this->position.y, this->position.x) = 1;
    cv::GaussianBlur(g, g, cv::Size(-1,-1), 2.5);
    double max;
    cv::minMaxIdx(g, NULL, &max);
    g /= max;

    cv::dft(g, this->G, cv::DFT_COMPLEX_OUTPUT);

    this->H1 = cv::Mat::zeros(this->G.rows, this->G.cols, CV_32FC2);
    this->H2 = cv::Mat::zeros(this->G.rows, this->G.cols, CV_32FC2);

    std::srand(std::time(NULL));

    for(int i = 0; i < 128; i++){
        cv::Mat A, aux;
        this->rndWarp(target, A);
        this->preProcess(A, aux);
        cv::dft(aux, A, cv::DFT_COMPLEX_OUTPUT);

        cv::mulSpectrums(this->conjug(G), A, aux, 0, false);
        this->H1 += aux;
        cv::mulSpectrums(this->conjug(A), A, aux, 0, false);
        this->H2 += aux;
    }
    this->updateKernel();
    this->updateTarget(target, false);
    this->updateKernel(target);
}

float Mosse::updateTarget(cv::Mat target, bool accept){
    cv::Mat image;
    preProcess(target, image);
    //cv::imshow("target", target);
    cv::Mat resp;
    float psr, confidence;
    double max;
    cv::Point2f pos;
    correlate(image, resp, pos, psr, max);
    //std::cout << "psr: " << psr << " " << max <<  "\n";
    //std::cout << "Pos: " << pos.x << " " << pos.y << "\n";
    if(psr > 3.0 || accept){
        ok = true;
        status = true;
        this->position = pos;
        confidence = max;
    } else {
        ok = false;
        status = false;
        this->position = cv::Point2f(0,0);
        confidence = 0;
    }

    return confidence;
}

void Mosse::updateKernel(cv::Mat target){
    if(ok){
        cv::Mat image;
        preProcess(target, image);
        cv::Mat A;
        cv::dft(image, A, cv::DFT_COMPLEX_OUTPUT);
        cv::Mat h1, h2;
        cv::mulSpectrums(this->conjug(G), A, h1, 0, false);
        cv::mulSpectrums(this->conjug(A), A, h2, 0, false);
        cv::Mat aux1, aux2;
        cv::multiply(this->H1, (1.0-RATE), aux1);
        cv::multiply(      h1,     (RATE), aux2);
        cv::add(aux1, aux2, this->H1);
        cv::multiply(this->H2, (1.0-RATE), aux1);
        cv::multiply(      h2,     (RATE), aux2);
        cv::add(aux1, aux2, this->H2);
        this->updateKernel();
    }
    //cv::imshow("correct", target);
    //cv::waitKey(0);
}

cv::Point2f Mosse::error(){
    //return position;
    return position;
}
