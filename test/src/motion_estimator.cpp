#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "imgproc.h"

int main()
{
    cv::Mat frame, img2;

    cv::VideoCapture cap("/home/corelaptop02/Thesis/samples/WIN_20220719_16_03_47_Pro.mp4");

    imageproc imgproc;

    if(!cap.isOpened())
    {
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while(1)
    {
        cap >> frame;

        imgproc.estimate(frame);

        if (frame.empty())
        {
            break;
        }
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}
