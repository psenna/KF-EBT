#include "flow.h"

Flow::Flow()
{

}

void Flow::computeTransform(){

}


void Flow::init(cv::Mat image, cv::Mat hsvImage, std::vector<cv::Point2f> rectangle, cv::Mat& Result){
    for (int j = 3; j > 1 ; j--) {
        for(int i = 0; i < j; i++){
            if(rectangle[i].x > rectangle[i+1].x){
                cv::Point2f aux = rectangle[i];
                rectangle[i] = rectangle[i+1];
                rectangle[i+1] = aux;
            }
        }
    }

    if(rectangle[0].y > rectangle[1].y){
        cv::Point2f aux = rectangle[0];
        rectangle[0] = rectangle[1];
        rectangle[1] = aux;
    }
    if(rectangle[2].y < rectangle[3].y){
        cv::Point2f aux = rectangle[2];
        rectangle[2] = rectangle[3];
        rectangle[3] = aux;
    }

    this->lastImage = image.clone();
    this->Lastrectangle = rectangle;


    float h = cv::norm(rectangle[0] - rectangle[1]);
    float w = cv::norm(rectangle[1] - rectangle[2]);

    sizeColor.width = w;
    sizeColor.height  = h;

    float border_h = h;
    float border_w = w;

    if(h < w){
        if(h < MINSIZE){
            w *= (MINSIZE/h);
            h = MINSIZE;
        }
    } else {
        if(w < MINSIZE){
            h *= (MINSIZE/w);
            w = MINSIZE;
        }
    }
    h *= KCF_PADDING;
    w *= KCF_PADDING;

    h = cv::getOptimalDFTSize(h);
    w = cv::getOptimalDFTSize(w);

    border_h = (h-border_h)/2;
    border_w = (w-border_w)/2;

    this->size.height =  (int) h;
    this->size.width = (int) w;

    cv::Point2f srcTri[3];
    srcTri[0] = rectangle[0];
    srcTri[1] = rectangle[1];
    srcTri[2] = rectangle[2];

    dstObj[0] = cv::Point2f(border_w, border_h);
    dstObj[1] = cv::Point2f(border_w, size.height-border_h);
    dstObj[2] = cv::Point2f(size.width-border_w, size.height-border_h);

    dstHSV[0] = cv::Point2f(0, 0);
    dstHSV[1] = cv::Point2f(0, sizeColor.height);
    dstHSV[2] = cv::Point2f(sizeColor.width, sizeColor.height);


    this->bases.push_back(cv::Point2f(border_w, border_h));
    this->bases.push_back(cv::Point2f(border_w, size.height-border_h));
    this->bases.push_back(cv::Point2f(size.width-border_w, size.height-border_h));
    this->bases.push_back(cv::Point2f(size.width-border_w, border_h));

    this->transformMatrix = cv::getAffineTransform(srcTri, dstObj);
    this->inverseTransformMatrix = cv::getAffineTransform(dstObj, srcTri);


    cv::Mat transf = cv::getAffineTransform(srcTri, dstHSV);
    cv::Mat hsvPatch;
    cv::warpAffine(hsvImage, hsvPatch, transf, this->sizeColor, cv::INTER_LINEAR);
    //colorHistogram.init(hsvPatch);

    cv::warpAffine(image, Result, this->transformMatrix, this->size, cv::INTER_LINEAR);
    errorRectangle = Lastrectangle;
    problem = false;

    speed = cv::Point2f(0,0);
    lastCenter = boxCenter(Lastrectangle);

    //cv::imshow("asd", Result);
    //cv::imshow("dffff", image);
    //cv::waitKey(0);
}

void Flow::getTransform(cv::Mat image, cv::Mat hsvImage, cv::Mat& Result){

    if(problem){
        lastHSVImage = realibleHSVImage;
        lastImage = realibleImage;
        Lastrectangle = realibleRectangle;
        lastKeypoints = realibleKeipoints;
        lastNoermedKeyPoints = realibleNoermedPoints;
    }

    // Select area for feature detection
    cv::Point rec[4];
    rec[0] = this->Lastrectangle[0];
    rec[1] = this->Lastrectangle[1];
    rec[2] = this->Lastrectangle[2];
    rec[3] = this->Lastrectangle[3];
    cv::Mat mask = cv::Mat::zeros(lastImage.size(), CV_8UC1);
    cv::fillConvexPoly(mask, rec, 4, 255);

    oldRectangle = Lastrectangle;

    std::vector<float> error;
    std::vector<unsigned char> status;

    // Remove points that drift from the target.
    /*cv::Mat im;
    cv::cvtColor(image, im, CV_GRAY2BGR);*/
    for(int i =lastKeypoints.size()-1; i >= 0; i--){
        unsigned char found = mask.at<unsigned char>(lastKeypoints[i].y, lastKeypoints[i].x);
        if(found == 0){
            lastKeypoints.erase(lastKeypoints.begin() + i);
            lastNoermedKeyPoints.erase(lastNoermedKeyPoints.begin()+i);
        }/* else {
            cv::circle(im, lastKeypoints[i], 1, cv::Scalar(0,0,255));
        }*/
    }

    normedPoints.clear();
    prevPoints.clear();
    nextPoints.clear();

    if(lastKeypoints.size() < MAXFEATURES){
        cv::goodFeaturesToTrack(lastImage, prevPoints, MAXFEATURES-lastKeypoints.size(), 0.001, 3, mask);
        if(prevPoints.size() > 0){
            cv::Point2f srcTri[3];
            srcTri[0] = this->Lastrectangle[0];
            srcTri[1] = this->Lastrectangle[1];
            srcTri[2] = this->Lastrectangle[2];
            this->transformMatrix = cv::getAffineTransform(srcTri, dstObj);
            cv::transform(prevPoints, normedPoints, transformMatrix);
        }
        lastKeypoints.insert(lastKeypoints.end(), prevPoints.begin(), prevPoints.end());
        lastNoermedKeyPoints.insert(lastNoermedKeyPoints.end(), normedPoints.begin(), normedPoints.end());
    }

    prevPoints = lastKeypoints;
    normedPoints = lastNoermedKeyPoints;
    lastNoermedKeyPoints.clear();
    lastKeypoints.clear();

    Result.release();
    if(prevPoints.size() > 0){

        std::vector<cv::Point2f> pointsBack;
        std::vector<unsigned char> status_back;
        std::vector<float> err_back;
        cv::calcOpticalFlowPyrLK(this->lastImage, image, prevPoints, nextPoints, status, error);

        cv::calcOpticalFlowPyrLK(image, this->lastImage, nextPoints, pointsBack, status_back, err_back);


        for(int i =prevPoints.size()-1; i >= 0; i--){

            float l2Error = cv::norm(prevPoints[i]-pointsBack[i]);

            if(!status[i] || !status_back[i] || l2Error > MAX_ERROR){
                nextPoints.erase(nextPoints.begin() + i);
                prevPoints.erase(prevPoints.begin() + i);
                normedPoints.erase(normedPoints.begin() + i);
            }/* else {
                cv::line(im, nextPoints[i], prevPoints[i], cv::Scalar(255,0,0));
            }*/
        }
        problem = true;
        float foundRate = (float)nextPoints.size()/(float)pointsBack.size();
        //cv::imshow("i", im);
        //std::cout << "Prev " << prevPoints.size() << "  " << foundRate << "\n";
        if(prevPoints.size() > MIN_FEATURES_FOUND && foundRate > ACCEPT_RATE){
            cv::Mat newTransform;
            std::vector<int> goodPoints;
            newTransform = estimateRigidTransform(prevPoints, nextPoints, goodPoints);
            if(!newTransform.empty()){
                std::vector<cv::Point2f> nextRectangle;
                cv::transform(Lastrectangle, nextRectangle, newTransform);

                cv::Point2f srcTri[3];
                srcTri[0] = nextRectangle[0];
                srcTri[1] = nextRectangle[1];
                srcTri[2] = nextRectangle[2];

                cv::Mat transf = cv::getAffineTransform(srcTri, dstHSV);
                cv::Mat hsvPatch;
                cv::warpAffine(hsvImage, hsvPatch, transf, this->sizeColor, cv::INTER_LINEAR);
                //problem = !(colorHistogram.compare(hsvPatch));
                if(!problem){
                    lastKeypoints.reserve(goodPoints.size());
                    lastNoermedKeyPoints.reserve(goodPoints.size());
                    for(int i = 0; i < goodPoints.size(); i++){
                        lastKeypoints.push_back(nextPoints[goodPoints[i]]);
                        lastNoermedKeyPoints.push_back(normedPoints[goodPoints[i]]);
                    }

                    std::vector<cv::Point2f> normed;
                    cv::transform(lastKeypoints, normed, transformMatrix);


                    for(int i =lastKeypoints.size()-1; i >= 0; i--){
                        float dx = fabs(normed[i].x - lastNoermedKeyPoints[i].x);
                        float dy = fabs(normed[i].y - lastNoermedKeyPoints[i].y);
                        if(dx > (size.width * 0.1) || dy > (size.height * 0.1)){
                            lastNoermedKeyPoints.erase(lastNoermedKeyPoints.begin()+i);
                            lastKeypoints.erase(lastKeypoints.begin()+i);
                        }
                    }

                    this->Lastrectangle = nextRectangle;
                    this->lastImage = image.clone();

                    /*if(colorHistogram.useKCF){
                        this->transformMatrix = cv::getAffineTransform(srcTri, dstObj);
                        this->inverseTransformMatrix = cv::getAffineTransform(dstObj, srcTri);
                        cv::warpAffine(image, Result, this->transformMatrix, this->size, cv::INTER_LINEAR);
                    }else{
                        Result.release();
                    }*/

                }
                //std::cout << lastKeypoints.size() << "\n";
            }
        }
    }

    lastHSVImage = hsvImage.clone();
    if(!problem){
        realibleImage = image.clone();
        realibleHSVImage = hsvImage.clone();
        realibleRectangle = Lastrectangle;
        realibleKeipoints = lastKeypoints;
        realibleNoermedPoints = lastNoermedKeyPoints;
    }

    if(problem){
        //std::cout << "SPEED: " << speed << "\n";
        cv::Point2f srcTri[3];
        srcTri[0] = this->errorRectangle[0] + speed*0.6;
        srcTri[1] = this->errorRectangle[1] + speed*0.6;
        srcTri[2] = this->errorRectangle[2] + speed*0.6;

        this->inverseTransformMatrix = cv::getAffineTransform(dstObj, srcTri);
        this->transformMatrix = cv::getAffineTransform(srcTri, dstObj);
        this->lastImage = image.clone();
        cv::warpAffine(image, Result, this->transformMatrix, this->size, cv::INTER_LINEAR);

    }
    //cv::imshow("res", Result);
    //cv::waitKey(0);
    //cv::imshow("dffff", image);
}

std::vector<cv::Point2f> Flow::getRectangle(){
    /*cv::Mat im;
    cv::cvtColor(this->lastImage, im, CV_GRAY2BGR);
    for(int i = 0; i < 4; i++){
        cv::line(im, this->Lastrectangle[i], this->Lastrectangle[(i+1)%4], cv::Scalar(255,0,0),1);
    }
    cv::imshow("kkkk", im);*/

    cv::Point2f center = boxCenter(Lastrectangle);
    speed = speed*0.2 + (center-lastCenter)*0.8;
    lastCenter = center;

    errorRectangle = Lastrectangle;
    return Lastrectangle;
}

std::vector<cv::Point2f> Flow::revertTransform(cv::Point2f correction, bool mosseStatus){

    if(problem){
        Lastrectangle = errorRectangle;
    }

    /*cv::Mat im;
    cv::cvtColor(this->lastImage, im, CV_GRAY2BGR);
    for(int i = 0; i < 4; i++){
        cv::line(im, this->Lastrectangle[i], this->Lastrectangle[(i+1)%4], cv::Scalar(255,0,0),1);
    }*/

    bool problemEstimate = true;

    std::vector<cv::Point2f> p;

    for(int i = 0; i < 4; i++){
        p.push_back(bases[i] + correction);
    }

    cv::transform(p, this->Lastrectangle, this->inverseTransformMatrix);

    if((correction.x != 0 || correction.y != 0) && nextPoints.size() > MIN_FEATURES_FOUND){
        //std::cout << "WOOOOOO\n";
        cv::Point2f centerOld = boxCenter(oldRectangle);
        cv::Point2f centerNew = boxCenter(Lastrectangle);
        std::vector<cv::Point2f> normPrev, normNext;
        normPrev.reserve(nextPoints.size());
        normNext.reserve(nextPoints.size());

        for(int i = 0; i < nextPoints.size(); i++){
            normPrev.push_back(prevPoints[i]-centerOld);
            normNext.push_back(nextPoints[i]-centerNew);
        }

        cv::Mat newTransform;
        std::vector<int> goodPoints;
        newTransform = estimateRigidTransformRS(normPrev, normNext, goodPoints);
        if(!newTransform.empty()){
            std::vector<cv::Point2f> rect;
            for(int i = 0; i < oldRectangle.size(); i++){
                rect.push_back(oldRectangle[i]-centerOld);
            }
            std::vector<cv::Point2f> nextRectangle;
            cv::transform(oldRectangle, nextRectangle, newTransform);

            for(int i = 0; i < oldRectangle.size(); i++){
                rect[i] += centerNew;
            }

            cv::Point2f srcTri[3];
            srcTri[0] = rect[0];
            srcTri[1] = rect[1];
            srcTri[2] = rect[2];

            cv::Mat transf = cv::getAffineTransform(srcTri, dstHSV);
            cv::Mat hsvPatch;
            cv::warpAffine(lastHSVImage, hsvPatch, transf, this->sizeColor, cv::INTER_LINEAR);
            //std::cout << "RS  ";
            //problemEstimate = !(colorHistogram.compare(hsvPatch));

            if(!problemEstimate){
                lastKeypoints.clear();
                lastNoermedKeyPoints.clear();
                lastKeypoints.reserve(goodPoints.size());
                lastNoermedKeyPoints.reserve(goodPoints.size());
                for(int i = 0; i < goodPoints.size(); i++){
                    lastKeypoints.push_back(nextPoints[goodPoints[i]]);
                    lastNoermedKeyPoints.push_back(normedPoints[goodPoints[i]]);
                }

                std::vector<cv::Point2f> normed;
                cv::transform(lastKeypoints, normed, transformMatrix);


                for(int i =lastKeypoints.size()-1; i >= 0; i--){
                    float dx = fabs(normed[i].x - lastNoermedKeyPoints[i].x);
                    float dy = fabs(normed[i].y - lastNoermedKeyPoints[i].y);
                    if(dx > (size.width * 0.1) || dy > (size.height * 0.1)){
                        lastNoermedKeyPoints.erase(lastNoermedKeyPoints.begin()+i);
                        lastKeypoints.erase(lastKeypoints.begin()+i);
                    }
                }

                this->Lastrectangle = rect;
                problem = false;
            }

        }

    }


    //std::cout << "Correction: "<< correction.x << "  " << correction.y << "\n";

    if(problemEstimate || problem){
        for(int i = 0; i < 4; i++){
            if(Lastrectangle[i].x < -20){
                Lastrectangle[i].x = -20;
            }
            if(Lastrectangle[i].y < -20){
                Lastrectangle[i].y = -20;
            }
            if(Lastrectangle[i].x > lastImage.cols+20){
                Lastrectangle[i].x = lastImage.cols +20;
            }
            if(Lastrectangle[i].y > lastImage.rows+20){
                Lastrectangle[i].y = lastImage.rows +20;
            }
        }

        cv::Point2f srcTri[3];
        srcTri[0] = Lastrectangle[0];
        srcTri[1] = Lastrectangle[1];
        srcTri[2] = Lastrectangle[2];

        cv::Mat transf = cv::getAffineTransform(srcTri, dstHSV);
        cv::Mat hsvPatch;
        cv::warpAffine(lastHSVImage, hsvPatch, transf, this->sizeColor, cv::INTER_LINEAR);
        //std::cout << "Recover  ";
        //problem = !(colorHistogram.compare(hsvPatch));
        if(!problem){
            realibleImage = lastImage.clone();
            realibleRectangle = Lastrectangle;
            realibleKeipoints.clear();
            realibleNoermedPoints.clear();
        }
    }

    /*for(int i = 0; i < 4; i++){
        cv::line(im, this->Lastrectangle[i], this->Lastrectangle[(i+1)%4], cv::Scalar(0,0,255),1);
    }
    cv::imshow("kkkk", im);*/

    cv::Point2f center = boxCenter(Lastrectangle);
    //cv::circle(im, center, 2, cv::Scalar(255, 0, 255), 2);
    //cv::circle(im, lastCenter, 2, cv::Scalar(255, 0, 0), 2);
    speed = speed*0.2 + (center-lastCenter)*0.8;
    lastCenter = center;

    errorRectangle = Lastrectangle;

    return Lastrectangle;
}

cv::Mat Flow::getMosseCorrection(){
    cv::Point2f srcTri[3];
    srcTri[0] = this->Lastrectangle[0];
    srcTri[1] = this->Lastrectangle[1];
    srcTri[2] = this->Lastrectangle[2];

    this->inverseTransformMatrix = cv::getAffineTransform(dstObj, srcTri);
    this->transformMatrix = cv::getAffineTransform(srcTri, dstObj);

    cv::Mat Result;
    cv::warpAffine(lastImage, Result, this->transformMatrix, this->size, cv::INTER_LINEAR);
    return Result;
}

std::vector<cv::Point2f> Flow::mosseSaveTheDay(cv::Point2f correction){

    return Lastrectangle;
}


void Flow::getRTMatrix( const cv::Point2f* a, const cv::Point2f* b, int count, cv::Mat& M)
{
    CV_Assert( M.isContinuous() );

    double sa[4][4]={{0.}}, sb[4]={0.}, m[4];
    cv::Mat A( 4, 4, CV_64F, sa ), B( 4, 1, CV_64F, sb );
    cv::Mat MM( 4, 1, CV_64F, m );

    for( int i = 0; i < count; i++ )
    {
        sa[0][0] += a[i].x*a[i].x + a[i].y*a[i].y;
        sa[0][2] += a[i].x;
        sa[0][3] += a[i].y;


        sa[2][1] += -a[i].y;
        sa[2][2] += 1;

        sa[3][0] += a[i].y;
        sa[3][1] += a[i].x;
        sa[3][3] += 1;

        sb[0] += a[i].x*b[i].x + a[i].y*b[i].y;
        sb[1] += a[i].x*b[i].y - a[i].y*b[i].x;
        sb[2] += b[i].x;
        sb[3] += b[i].y;
    }

    sa[1][1] = sa[0][0];
    sa[2][1] = sa[1][2] = -sa[0][3];
    sa[3][1] = sa[1][3] = sa[2][0] = sa[0][2];
    sa[2][2] = sa[3][3] = count;
    sa[3][0] = sa[0][3];

    cv::solve( A, B, MM, cv::DECOMP_EIG );

    double* om = M.ptr<double>();
    om[0] = om[4] = m[0];
    om[1] = -m[1];
    om[3] = m[1];
    om[2] = m[2];
    om[5] = m[3];
}

cv::Mat Flow::estimateRigidTransform( cv::InputArray src1, cv::InputArray src2, std::vector<int>& bestPoints)
{
    cv::Mat M(2, 3, CV_64F), A = src1.getMat(), B = src2.getMat();

    const int RANSAC_MAX_ITERS = 500;
    const int RANSAC_SIZE0 = 3;
    const double RANSAC_GOOD_RATIO = 0.50;

    std::vector<cv::Point2f> pA, pB;
    std::vector<int> good_idx;
    std::vector<uchar> status;

    double scale = 1.0;
    int i, j, k, k1;

    cv::RNG rng((uint64)-1);
    int good_count = 0;

    int count = A.checkVector(2);

    if( count > 0 )
    {
        A.reshape(2, count).convertTo(pA, CV_32F);
        B.reshape(2, count).convertTo(pB, CV_32F);
    }

    good_idx.resize(count);

    if( count < RANSAC_SIZE0 )
        return cv::Mat();

    cv::Rect brect = cv::boundingRect(pB);

    // RANSAC stuff:
    // 1. find the consensus
    float minError = std::max(brect.width,brect.height);
    for( k = 0; k < RANSAC_MAX_ITERS; k++ )
    {
        int idx[RANSAC_SIZE0];
        cv::Point2f a[RANSAC_SIZE0];
        cv::Point2f b[RANSAC_SIZE0];

        // choose random 3 non-complanar points from A & B
        for( i = 0; i < RANSAC_SIZE0; i++ )
        {
            for( k1 = 0; k1 < RANSAC_MAX_ITERS; k1++ )
            {
                idx[i] = rng.uniform(0, count);

                for( j = 0; j < i; j++ )
                {
                    if( idx[j] == idx[i] )
                        break;
                    // check that the points are not very close one each other
                    if( fabs(pA[idx[i]].x - pA[idx[j]].x) +
                            fabs(pA[idx[i]].y - pA[idx[j]].y) < FLT_EPSILON )
                        break;
                    if( fabs(pB[idx[i]].x - pB[idx[j]].x) +
                            fabs(pB[idx[i]].y - pB[idx[j]].y) < FLT_EPSILON )
                        break;
                }

                if( j < i )
                    continue;

                if( i+1 == RANSAC_SIZE0 )
                {
                    // additional check for non-complanar vectors
                    a[0] = pA[idx[0]];
                    a[1] = pA[idx[1]];
                    a[2] = pA[idx[2]];

                    b[0] = pB[idx[0]];
                    b[1] = pB[idx[1]];
                    b[2] = pB[idx[2]];

                    double dax1 = a[1].x - a[0].x, day1 = a[1].y - a[0].y;
                    double dax2 = a[2].x - a[0].x, day2 = a[2].y - a[0].y;
                    double dbx1 = b[1].x - b[0].x, dby1 = b[1].y - b[0].y;
                    double dbx2 = b[2].x - b[0].x, dby2 = b[2].y - b[0].y;
                    const double eps = 0.01;

                    if( fabs(dax1*day2 - day1*dax2) < eps*std::sqrt(dax1*dax1+day1*day1)*std::sqrt(dax2*dax2+day2*day2) ||
                            fabs(dbx1*dby2 - dby1*dbx2) < eps*std::sqrt(dbx1*dbx1+dby1*dby1)*std::sqrt(dbx2*dbx2+dby2*dby2) )
                        continue;
                }
                break;
            }

            if( k1 >= RANSAC_MAX_ITERS )
                break;
        }

        if( i < RANSAC_SIZE0 )
            continue;

        // estimate the transformation using 3 points
        getRTMatrix( a, b, 3, M);
        float err = 0;
        const double* m = M.ptr<double>();
        for( i = 0, good_count = 0; i < count; i++ )
        {
            if( std::fabs( m[0]*pA[i].x + m[1]*pA[i].y + m[2] - pB[i].x ) +
                    std::fabs( m[3]*pA[i].x + m[4]*pA[i].y + m[5] - pB[i].y ) < std::max(brect.width,brect.height)*0.035){
                good_idx[good_count++] = i;
                err += std::fabs( m[0]*pA[i].x + m[1]*pA[i].y + m[2] - pB[i].x ) + std::fabs( m[3]*pA[i].x + m[4]*pA[i].y + m[5] - pB[i].y);
            }
        }
        err /= good_count;
        if( good_count >= count*RANSAC_GOOD_RATIO && err < minError){
            //std::cout << "RANSAC : " << k << "  " << err << " Count " << good_count << "\n";
            minError = err;
            bestPoints = good_idx;
            bestPoints.resize(good_count);
        }
    }

    if( bestPoints.size() == 0 )
        return cv::Mat();
    good_count = bestPoints.size();
    good_idx = bestPoints;

    if( good_count < count )
    {
        for( i = 0; i < good_count; i++ )
        {
            j = good_idx[i];
            pA[i] = pA[j];
            pB[i] = pB[j];
        }
    }

    getRTMatrix( &pA[0], &pB[0], good_count, M );
    M.at<double>(0, 2) /= scale;
    M.at<double>(1, 2) /= scale;

    return M;
}

void Flow::getRSMatrix( const cv::Point2f* a, const cv::Point2f* b, int count, cv::Mat& M)
{
    CV_Assert( M.isContinuous());

    double sa[2][2]={{0.}}, sb[2]={0.}, m[2];
    cv::Mat A( 2, 2, CV_64F, sa ), B( 2, 1, CV_64F, sb );
    cv::Mat MM( 2, 1, CV_64F, m );

    for( int i = 0; i < count; i++ )
    {
        sa[0][0] += a[i].x*a[i].x + a[i].y*a[i].y;

        sb[0] += a[i].x*b[i].x + a[i].y*b[i].y;
        sb[1] += a[i].x*b[i].y - a[i].y*b[i].x;
    }

    sa[1][1] = sa[0][0];

    cv::solve( A, B, MM, cv::DECOMP_EIG);

    double* om = M.ptr<double>();
    om[0] = om[4] = m[0];
    om[1] = -m[1];
    om[3] = m[1];
    om[2] = 0;
    om[5] = 0;
}

cv::Mat Flow::estimateRigidTransformRS( cv::InputArray src1, cv::InputArray src2, std::vector<int>& bestPoints){
    cv::Mat M(2, 3, CV_64F), A = src1.getMat(), B = src2.getMat();

    const int RANSAC_MAX_ITERS = 500;
    const int RANSAC_SIZE0 = 3;
    const double RANSAC_GOOD_RATIO = 0.50;

    std::vector<cv::Point2f> pA, pB;
    std::vector<int> good_idx;
    std::vector<uchar> status;

    double scale = 1.0;
    int i, j, k, k1;

    cv::RNG rng((uint64)-1);
    int good_count = 0;

    int count = A.checkVector(2);

    if( count > 0 )
    {
        A.reshape(2, count).convertTo(pA, CV_32F);
        B.reshape(2, count).convertTo(pB, CV_32F);
    }

    good_idx.resize(count);

    if( count < RANSAC_SIZE0 )
        return cv::Mat();

    cv::Rect brect = cv::boundingRect(pB);

    // RANSAC stuff:
    // 1. find the consensus
    float minError = std::max(brect.width,brect.height);
    for( k = 0; k < RANSAC_MAX_ITERS; k++ )
    {
        int idx[RANSAC_SIZE0];
        cv::Point2f a[RANSAC_SIZE0];
        cv::Point2f b[RANSAC_SIZE0];

        // choose random 3 non-complanar points from A & B
        for( i = 0; i < RANSAC_SIZE0; i++ )
        {
            for( k1 = 0; k1 < RANSAC_MAX_ITERS; k1++ )
            {
                idx[i] = rng.uniform(0, count);

                for( j = 0; j < i; j++ )
                {
                    if( idx[j] == idx[i] )
                        break;
                    // check that the points are not very close one each other
                    if( fabs(pA[idx[i]].x - pA[idx[j]].x) +
                            fabs(pA[idx[i]].y - pA[idx[j]].y) < FLT_EPSILON )
                        break;
                    if( fabs(pB[idx[i]].x - pB[idx[j]].x) +
                            fabs(pB[idx[i]].y - pB[idx[j]].y) < FLT_EPSILON )
                        break;
                }

                if( j < i )
                    continue;

                if( i+1 == RANSAC_SIZE0 )
                {
                    // additional check for non-complanar vectors
                    a[0] = pA[idx[0]];
                    a[1] = pA[idx[1]];
                    a[2] = pA[idx[2]];

                    b[0] = pB[idx[0]];
                    b[1] = pB[idx[1]];
                    b[2] = pB[idx[2]];

                    double dax1 = a[1].x - a[0].x, day1 = a[1].y - a[0].y;
                    double dax2 = a[2].x - a[0].x, day2 = a[2].y - a[0].y;
                    double dbx1 = b[1].x - b[0].x, dby1 = b[1].y - b[0].y;
                    double dbx2 = b[2].x - b[0].x, dby2 = b[2].y - b[0].y;
                    const double eps = 0.01;

                    if( fabs(dax1*day2 - day1*dax2) < eps*std::sqrt(dax1*dax1+day1*day1)*std::sqrt(dax2*dax2+day2*day2) ||
                            fabs(dbx1*dby2 - dby1*dbx2) < eps*std::sqrt(dbx1*dbx1+dby1*dby1)*std::sqrt(dbx2*dbx2+dby2*dby2) )
                        continue;
                }
                break;
            }

            if( k1 >= RANSAC_MAX_ITERS )
                break;
        }

        if( i < RANSAC_SIZE0 )
            continue;

        // estimate the transformation using 3 points
        getRSMatrix(a, b, 3, M);
        float err = 0;
        const double* m = M.ptr<double>();
        for( i = 0, good_count = 0; i < count; i++ )
        {
            if( std::fabs( m[0]*pA[i].x + m[1]*pA[i].y + m[2] - pB[i].x ) +
                    std::fabs( m[3]*pA[i].x + m[4]*pA[i].y + m[5] - pB[i].y ) < std::max(brect.width,brect.height)*0.05){
                good_idx[good_count++] = i;
                err += std::fabs( m[0]*pA[i].x + m[1]*pA[i].y + m[2] - pB[i].x ) + std::fabs( m[3]*pA[i].x + m[4]*pA[i].y + m[5] - pB[i].y);
            }
        }
        err /= good_count;
        if( good_count >= count*RANSAC_GOOD_RATIO && err < minError){
            //std::cout << "RANSAC : " << k << "  " << err << " Count " << good_count << "\n";
            minError = err;
            bestPoints = good_idx;
            bestPoints.resize(good_count);
        }
    }

    if( bestPoints.size() == 0 )
        return cv::Mat();
    good_count = bestPoints.size();
    good_idx = bestPoints;

    if( good_count < count )
    {
        for( i = 0; i < good_count; i++ )
        {
            j = good_idx[i];
            pA[i] = pA[j];
            pB[i] = pB[j];
        }
    }

    getRSMatrix( &pA[0], &pB[0], good_count, M );
    M.at<double>(0, 2) /= scale;
    M.at<double>(1, 2) /= scale;

    return M;
}



void Flow::rotateBox(std::vector<cv::Point2f>& box, float angle){
    cv::Point2f center = (box[0]+box[1]+box[2]+box[3])/4;
    cv::Point2f p;

    for(int i = 0; i < 4; i++){
        p.x = ((box[i].x-center.x)*cos(angle)) - ((box[i].y-center.y)*sin(angle)) + center.x;
        p.y = ((box[i].x-center.x)*sin(angle)) + ((box[i].y-center.y)*cos(angle)) + center.y;
        box[i] = p;
    }
}

void Flow::scaleBox(std::vector<cv::Point2f>& box, float scale){
    cv::Point2f center = (box[0]+box[1]+box[2]+box[3])/4;
    cv::Point2f p;

    for(int i = 0; i < 4; i++){
        p = box[i]-center;
        p*= scale;
        p+= center;
        box[i] = p;
    }
}

void Flow::translateBox(std::vector<cv::Point2f>& box, cv::Point2f distance){
    cv::Point2f p;
    for(int i = 0; i < 4; i++){
        box[i]+=distance;
    }
}

cv::Point2f Flow::boxCenter(std::vector<cv::Point2f> box){
    return (box[0]+box[1]+box[2]+box[3])/4;
}
