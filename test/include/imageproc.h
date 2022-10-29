#ifndef IMGPROC_H
#define IMGPROC_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


#include "odom.h"

static const std::string OPENCV_WINDOW = "Image window";

class imageproc
{
public:

    imageproc();

    ~imageproc();

    void estimate(cv::Mat img);
    void imageCb(const sensor_msgs::ImageConstPtr &msg);

    odom::Frame key_frame;
    odom::Frame frame;
    odom::feature_detector detect;

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;
    cv_bridge::CvImage out_img;

    cv::Mat img, img_gray;

};

#endif // IMGPROC_H
