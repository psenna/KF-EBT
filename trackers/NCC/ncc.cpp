#include "ncc.h"

void NCCTracker::init(cv::Mat &img, cv::Rect rect){
    p_window = MAX(rect.width, rect.height) * 2;

    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    int left = MAX(rect.x, 0);
    int top = MAX(rect.y, 0);

    int right = MIN(rect.x + rect.width, gray.cols - 1);
    int bottom = MIN(rect.y + rect.height, gray.rows - 1);

    cv::Rect roi(left, top, right - left, bottom - top);

    gray(roi).copyTo(p_template);

    p_position.x = (float)rect.x + (float)rect.width / 2;
    p_position.y = (float)rect.y + (float)rect.height / 2;

    p_size = cv::Size2f(rect.width, rect.height);

}

double NCCTracker::track(cv::Mat img){

    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    float left = MAX(round(p_position.x - (float)p_window / 2), 0);
    float top = MAX(round(p_position.y - (float)p_window / 2), 0);

    float right = MIN(round(p_position.x + (float)p_window / 2), gray.cols - 1);
    float bottom = MIN(round(p_position.y + (float)p_window / 2), gray.rows - 1);

    cv::Rect roi((int) left, (int) top, (int) (right - left), (int) (bottom - top));

    if (roi.width < p_template.cols || roi.height < p_template.rows) {
        return 0;
    }

    cv::Mat matches;
    cv::Mat cut = gray(roi);

    cv::matchTemplate(cut, p_template, matches, CV_TM_CCOEFF_NORMED);

    cv::Point matchLoc;
    double max;
    cv::minMaxLoc(matches, NULL, &max, NULL, &matchLoc, cv::Mat());

    p_position.x = left + matchLoc.x + (float)p_size.width / 2;
    p_position.y = top + matchLoc.y + (float)p_size.height / 2;

    return max;

}
