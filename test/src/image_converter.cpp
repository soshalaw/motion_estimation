#include "image_converter.h"

image_converter::image_converter()
: it_(nh_)
{
    image_sub = it_.subscribe("/usb_cam/image", 1 , &image_converter::imageCb, this);
    cam_pose = nh_.subscribe("/vrpn_client_node/minicar_cam/pose", 1, &image_converter::update_pose, this);
    cv::namedWindow(OPENCV_WINDOW);
    
    img_counter = 0;
}

image_converter::~image_converter()
{
    //cv::destroyWindow(OPENCV_WINDOW);
}

void image_converter::imageCb(const sensor_msgs::ImageConstPtr& msg)  //function to convert data to cv::Mat
{
    cv::Mat frame;
    cv_bridge::CvImagePtr cv_ptr;

    time_stamp_img_init = msg->header.stamp;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

        img = cv_ptr->image;

        /*if(time_stamp_img_init > time_stamp_img)
        {*/
            write_data();
        //}
 /*      else
        {
            ROS_ERROR_STREAM("Old data");
        }*/
    }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }

    time_stamp_img = time_stamp_img_init;
}

void image_converter::update_pose(const geometry_msgs::PoseStampedPtr& msg)
{
    time_stamp_pose_init = msg->header.stamp;

    cam_x = msg->pose.position.x;
    cam_y = msg->pose.position.y;
    cam_z = msg->pose.position.z;

    theta_qx = msg->pose.orientation.x;
    theta_qy = msg->pose.orientation.y;
    theta_qz = msg->pose.orientation.z;
    theta_qw = msg->pose.orientation.w;

    tf::Quaternion q2(theta_qx, theta_qy, theta_qz, theta_qw);
    tf::Matrix3x3 m2(q2);

    m2.getRPY(theta_x, theta_y, theta_z);

    time_stamp_pose = time_stamp_pose_init;
}

void image_converter::write_data()
{
    std::string name = "/home/corelaptop02/Soshala_stuff/Thesis/samples/cam_data_test/" + std::to_string(img_counter) + ".png" ;
    
    tf = std::ofstream("/home/corelaptop02/Soshala_stuff/Thesis/samples/cam_data_test/tf" + std::to_string(img_counter) + ".txt");

    dtime = std::abs((time_stamp_pose_init - time_stamp_img_init).toSec());

    if(!img.empty())
    {
        cv::imwrite(name,img);

        if (tf.is_open())
        {
            tf << cam_x << " " << cam_y << " " << cam_z << " " << theta_x << " " << theta_y << " " << theta_z << " " << dtime  << " " << time_stamp_img_init << std::endl;    
        }

        img_counter ++;

        tf.close();
    }


}


