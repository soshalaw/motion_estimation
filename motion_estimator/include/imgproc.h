#ifndef IMGPROC_H
#define IMGPROC_H

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "odom.h"

class imageproc
{
public:
    imageproc();
    void estimate(cv::Mat frame);

    odom::Frame current_frame;
    odom::Frame prev_frame;
    odom::feature_detector detect;
};

#endif // IMGPROC_H
