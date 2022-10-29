#ifndef ODOM_H
#define ODOM_H

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <numeric>

namespace odom {

class Frame {
    public:
        cv::Mat image_gray;
        //std::vector<cv::Point2f> feature_points;
        std::vector<cv::KeyPoint> keypoints;

        /*cv::Mat R;
        cv::Mat t;
        cv::Mat camera_pose;
        cv::Mat desc;*/

        Frame();
        Frame(cv::Mat _image);
    };

class feature_detector
{
public:
    feature_detector();

    void orb_descriptor(cv::Mat frame, cv::Mat desc, std::vector<cv::Point2f> &feature_points);
    void estimate_matching(cv::Mat frame1, cv::Mat frame2);
    void estimate_opticalflw(cv::Mat frame2);
    void set_keyframe(cv::Mat frame);
    void goodMatchesFeatures(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, cv::Mat mask);
    std::vector<cv::KeyPoint> ssc(std::vector<cv::KeyPoint> keyPoints, int numRetPoints,float tolerance, int cols, int rows);
    //void recover_pose()

private:
    cv::Ptr<cv::ORB> orb;
    cv::BFMatcher matcher;

    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<cv::KeyPoint> keypoints_keyframe, keypoints_frame, keypoints_keyframe_, keypoints_frame_;
    cv::Mat image_matches, image_resize, key_frame;
    std::vector<cv::Point2f> points_frame, points_keyframe;
    std::vector<cv::Point2f> goodmatches;
    cv::Mat desc1 , desc2, E;
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    std::vector<float> responseVector;
    std::vector<cv::KeyPoint> keyPointsSorted;

    cv::Mat R;
    cv::Mat t;
    cv::Mat camera_pose, mask;
};

}

#endif // ODOM_H
