#include "imageproc.h"

imageproc::imageproc()
: it_(nh_)
{
      image_sub = it_.subscribe("usb_cam/image", 1 , &imageproc::imageCb, this);
      image_pub = it_.advertise("camera/image_raw",1000);
      cv::namedWindow(OPENCV_WINDOW);
}

imageproc::~imageproc()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void imageproc::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exceprion: %s",e.what());
        return;
    }

    img = cv_ptr->image;

    estimate(img);

}

void imageproc::estimate(cv::Mat img)
{
    frame = odom::Frame(img);

    if(key_frame.image_gray.empty())
    {
        key_frame = frame;
        detect.set_keyframe(frame.image_gray);
        ROS_INFO("Setting keyframe");
    }

    detect.estimate_opticalflw(frame.image_gray);

    //prev_frame = current_frame;

    /*cv::imshow("image", prev_frame.image);

    cv::waitKey(1);*/

}
