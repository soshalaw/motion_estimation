#ifndef IMAGE_BRIDGE_H
#define IMAGE_BRIDGE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";

class image_bridge
{
public:
    image_bridge();

    ~image_bridge();

    void imageCbA(const sensor_msgs::ImageConstPtr &msg);

    void imageCbB(const sensor_msgs::ImageConstPtr &msg);

    void merge();


private:

    cv::Mat img1, img2;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub1, image_sub2;
};

#endif // IMAGE_BRIDGE_H
