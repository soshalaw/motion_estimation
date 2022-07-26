#ifndef IMAGE_BRIDGE_H
#define IMAGE_BRIDGE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ocam_functions.h"

static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";

class image_bridge
{
public:
    image_bridge();

    ~image_bridge();

    void imageCbA(const sensor_msgs::ImageConstPtr &msg);

    void imageCbB(const sensor_msgs::ImageConstPtr &msg);

    void panorama();

    void cuboidal_proj();

private:

    cv::Mat img1, img2, new_img, res, pan_1, pan_2, proj;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub1, image_sub2;

    std::array<double,3> c;

    //calibration data camera 01 & 02
    std::vector<double> cam1_invpol, cam2_invpol;
    std::vector<double> cam1_pol, cam2_pol;
    double cam1_yc, cam2_yc;
    double cam1_xc, cam2_xc;

    double cam1_c, cam2_c;
    double cam1_d, cam2_d;
    double cam1_e, cam2_e;

    ocam_functions image_proc;

    //panoramic image vertical fov
    double delta_min, delta_max;

    //projected image fov
    double theta_min, theta_max, alpha_min, alpha_max;
    int proj_cols, proj_rows;
    double H_res;
    
    //projected image vertical fov
    std::vector<std::vector<double>> centers;  //definition of the 8 image centers
    
};

#endif // IMAGE_BRIDGE_H
