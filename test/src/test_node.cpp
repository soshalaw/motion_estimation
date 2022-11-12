#include <iostream>
#include <ros/ros.h>

#include "image_converter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    image_converter img;
    ros::spin();

    cv::destroyAllWindows();
    return 0;
}
