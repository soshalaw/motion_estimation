#include "image_bridge.h"

image_bridge::image_bridge()
: it_(nh_)
{
    image_sub1 = it_.subscribe("usb_cam1/image_raw", 1 , &image_bridge::imageCbA, this);
    image_sub2 = it_.subscribe("usb_cam2/image_raw", 1 , &image_bridge::imageCbB, this);
    cv::namedWindow(OPENCV_WINDOW1);
}

image_bridge::~image_bridge()
{
     cv::destroyWindow(OPENCV_WINDOW1);
     //cv::destroyWindow(OPENCV_WINDOW2);
}

void image_bridge::imageCbA(const sensor_msgs::ImageConstPtr &msg)
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

    img1 = cv_ptr->image;

    //cv::imshow(OPENCV_WINDOW1, img1);

    //cv::waitKey(1);

}


void image_bridge::imageCbB(const sensor_msgs::ImageConstPtr &msg)
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

    img2 = cv_ptr->image;

    merge();

}

void image_bridge::merge()
{
    cv::Mat new_img, res;

    new_img = cv::Mat::zeros(img1.rows, (img1.cols+img2.cols), img1.type());

    img1.copyTo(new_img(cv::Rect(0,0,img1.cols,img1.rows)));
    img2.copyTo(new_img(cv::Rect(img1.cols,0,img1.cols,img1.rows)));;

    cv::resize(new_img, res, cv::Size(new_img.cols/5, new_img.rows/5));

    cv::imshow(OPENCV_WINDOW1, res);

    cv::waitKey(1);

}
