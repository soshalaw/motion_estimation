#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>


class feature_detector
{
public:
    feature_detector();

    void orb_descriptor(cv::Mat frame);
    void set_keyframe(cv::Mat frame);

private:

    cv::Ptr<cv::ORB> orb;
    cv::Ptr<cv::DescriptorMatcher> matcher_br;
    cv::Ptr<cv::DescriptorMatcher> matcher_fann;

    std::vector<cv::KeyPoint> kp_kfram, kp2;
    cv::Mat desc1, desc2;

    std::vector<cv::DMatch> matches;

    cv::Mat image_matches, image_resize, key_frame;
};

#endif // FEATURE_DETECTOR_H
