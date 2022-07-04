#ifndef OCAM_FUNCTIONS_H
#define OCAM_FUNCTIONS_H

#include "math.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>


class ocam_functions
{
public:
    ocam_functions();

    void world2cam(double point2D[2], double point3D[3], double xc, double yc, double c, double d, double e, std::vector<double> invpol);

    cv::Mat slice(cv::Mat M, double c[3], double theta_min, double theta_max, double delta_min, double delta_max);


private:
    cv::Mat img, ImgPointsx, ImgPointsy;  // definition of matrices for the output image and remapping

    double x, y, z, cos_alpha, x_, y_, z_;
    double planer_coords[3];
    double points2D[2];
    double cp_x, cp_y, cp_z, modx, mody;

    //calibration data camera 01
    std::vector<double> cam1_invpol;
    std::list<double> cam1_invpol_list;
    std::vector<double> cam1_pol;
    std::list<double> cam1_pol_list;
    double cam1_yc;
    double cam1_xc;

    double cam1_c;
    double cam1_d;
    double cam1_e;

    //calibration data camera 02
    std::vector<double> cam2_invpol;
    std::list<double> cam2_invpol_list;
    std::vector<double> cam2_pol;
    std::list<double> cam2_pol_list;
    double cam2_yc;
    double cam2_xc;

    double cam2_c;
    double cam2_d;
    double cam2_e;


    int H_res; // length of the output image

    int mode;

};

#endif // OCAM_FUNCTIONS_H
