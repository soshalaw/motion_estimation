#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include "image_bridge.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_bridge_node");
    image_bridge bridge;
    ros::spin();
    return 0;
}
