#ifndef OCAM_FUNCTIONS_H
#define OCAM_FUNCTIONS_H

#include "math.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


class ocam_functions
{
public:
    ocam_functions();

    void world2cam(double point2D[2], double point3D[3]);

    cv::Mat slice(cv::Mat M, double c[3], double theta_min, double theta_max, double delta_min, double delta_max);

private:
    cv::Mat img, ImgPointsx, ImgPointsy;  // definition of matrices for the output image and remapping

    double x, y, z, cos_alpha, x_, y_, z_;
    double planer_coords[3];
    double points2D[2];
    double cp_x, cp_y, cp_z, modx, mody;

    //calibration data camera 01
    std::vector<double> invpol = {-1.075233654325322e+02, 4.704007234612547e+02, -7.039759405818603e+02, 2.838316240089585e+02, -9.038676504203400e+02, 1.606691263137671e+03};
    std::vector<double> pol = {1.145882288545091e+03 ,0 ,-3.630427146836845e-04 ,1.047936476714481e-07 ,-1.000973064316358e-10};
    double yc = 1.307765250016426e+03;
    double xc = 1.647330746606595e+03;

    double c = 1;
    double d = 0;
    double e = 0;

    //calibration data camera 02 TODO
   /* std::vector<double> invpol = {-1.075233654325322e+02, 4.704007234612547e+02, -7.039759405818603e+02, 2.838316240089585e+02, -9.038676504203400e+02, 1.606691263137671e+03};
    std::vector<double> pol = {1.145882288545091e+03 ,0 ,-3.630427146836845e-04 ,1.047936476714481e-07 ,-1.000973064316358e-10};
    double yc = 1.307765250016426e+03;
    double xc = 1.647330746606595e+03;

    double c = 1;
    double d = 0;
    double e = 0;*/


    int H_res = 1024; // length of the output image

    int mode = 0;

};

#endif // OCAM_FUNCTIONS_H
