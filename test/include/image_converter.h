
#include <array>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

static const std::string OPENCV_WINDOW = "Image window";

class image_converter
{
public:

    image_converter();

    ~image_converter();

    void imageCb(const sensor_msgs::ImageConstPtr& msg);  //function to convert data to cv::Mat

    void update_pose(const geometry_msgs::PoseStampedPtr& msg);

    void write_data();
    
    inline bool is_same (ros::Time tstamp1, ros::Time tstamp2)
    {
        return (tstamp1 == tstamp2);
    }


private :
    cv::Mat img, frame;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub;
    ros::Subscriber cam_pose;

    int img_counter;

    ros::Time time_stamp_img, time_stamp_img_init;
    ros::Time time_stamp_pose, time_stamp_pose_init;
    double dtime_pose, dtime_image, dtime;

    double cam_x, cam_y, cam_z, theta_qx, theta_qy, theta_qz, theta_qw, theta_x, theta_y, theta_z; 
    //double cam_x, cam_y, cam_z, theta_qx, theta_qy, theta_qz, theta_qw;
    std::ofstream tf;
    
};
