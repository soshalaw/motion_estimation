#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

#include "odom.h"


#define START_FRAME 50
#define MAX_FRAME 500//4540

#define DISPLAY3D true
#define CLOUD_NAME "Triangulated Point Cloud"


int main()
{
    /*bool initial = true;
    int initial_x = 0;
    int initial_y = 0;
    int initial_z = 0;

    double scale = 1.00;
    char filename1[200];
    char filename2[200];
    sprintf(filename1, "%s/%06d.png", dataset_directory.c_str(), 0);
    sprintf(filename2, "%s/%06d.png", dataset_directory.c_str(), 1);*/

    /*Camera intristic parameter matrix
    K = (cv::Mat_<double>(3,3) <<   7.188560000000e+02, 0.000000000000e+00, 6.071928000000e+02,
                                            0.000000000000e+00, 7.188560000000e+02, 1.852157000000e+02,
                                            0.000000000000e+00, 0.000000000000e+00, 1.000000000000e+00);*/

   /* cout << "K:" << endl;
    cout << K << endl;*/

    odom::feature_detector detect;
    odom::Frame current_frame;
    odom::Frame previous_frame;

   // slam::SharedData shared_data(K);

    // cout << focal << endl;
    // cout << pp << endl;

    char filename[100];

    std::string dataset_directory = "/home/corelaptop02/Thesis/samples/dataset/sequences/00/image_0";
    namedWindow( "Road facing camera", cv::WINDOW_AUTOSIZE );

    //Mat traj = Mat::zeros(600, 600, CV_8UC3);

    //std::vector<Eigen::Affine3f> camera_poses;

    for(int numFrame=START_FRAME; numFrame < MAX_FRAME; numFrame++) {
        std::sprintf(filename, "%s/%06d.png", dataset_directory.c_str(), numFrame);

        current_frame = odom::Frame(cv::imread(filename, cv::IMREAD_COLOR));

        if ( !current_frame.image_gray.data) {
            std::cout<< " --(!) Error reading images " << std::endl; return -1;
        }

        if (previous_frame.image_gray.empty()){
            // initial
            previous_frame = current_frame;
            continue;
        }

        cv::Mat E, mask;

        detect.estimate_opticalflw(current_frame.image_gray, previous_frame.image_gray);

        cv::imshow( "Road facing camera", current_frame.image_gray );
        //imshow( "Trajectory", traj_mirror);

        if (cv::waitKey(1) == 'n')
            break;

        previous_frame = current_frame;

    }
    cv::destroyAllWindows();

    return 0;
}
