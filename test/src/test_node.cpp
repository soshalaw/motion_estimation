#include <iostream>
#include <ros/ros.h>

#include "imageproc.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_bridge_node");
    imageproc imgproc;
    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
