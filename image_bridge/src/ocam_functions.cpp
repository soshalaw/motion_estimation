#include "ocam_functions.h"

ocam_functions::ocam_functions()
{
    ros::param::get("cam1_invpol", cam1_invpol);
    ros::param::get("cam1_mapcoef", cam1_pol);;
    ros::param::get("cam1_yc", cam1_yc);
    ros::param::get("cam1_xc", cam1_xc);
    ros::param::get("cam1_c", cam1_c);
    ros::param::get("cam1_d", cam1_d);
    ros::param::get("cam1_e", cam1_e);

    ros::param::get("cam2_invpol", cam2_invpol);
    ros::param::get("cam2_mapcoef", cam2_pol);
    ros::param::get("cam2_yc", cam2_yc);
    ros::param::get("cam2_xc", cam2_xc);
    ros::param::get("cam2_c", cam2_c);
    ros::param::get("cam2_d", cam2_d);
    ros::param::get("cam2_e", cam2_e);

}

void ocam_functions::world2cam(double point2D[2], double point3D[3], double xc, double yc, double c, double d, double e, std::vector<double> invpol, std::vector<double> pol)
{
     double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
     double theta       = atan(point3D[2]/norm);
     int length_invpol  = 6;
     double t, t_i;
     double rho, x, y;
     double invnorm;
     int i;

     if (norm != 0)
     {
        invnorm = 1/norm;
        t  = theta;
        rho = invpol[0];
        t_i = 1;

        for (i = 1; i < length_invpol; i++)
        {
          rho *= t;
          rho += invpol[i];
        y = point3D[1]*invnorm*rho;
        }

        x = point3D[0]*invnorm*rho;

        point2D[0] = x*c + y*d + xc;
        point2D[1] = x*e + y   + yc;
     }
     else
     {
        point2D[0] = xc;
        point2D[1] = yc;
     }
}

cv::Mat ocam_functions::slice(cv::Mat M, double c[3], double theta_min, double theta_max, double delta_min, double delta_max)
{
    double alpha = theta_max - theta_min;

    double gamma = delta_max - delta_min;

    double theta = 0;

    int V_res = tan(gamma/2)*H_res/tan(alpha/2);

    img.create(V_res, H_res,M.type());
    ImgPointsx.create(img.size(), CV_32FC1);
    ImgPointsy.create(img.size(), CV_32FC1);

    if (mode == 1)
    {
        c[0] = cp_x = sin(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
        c[1] = cp_y = cos(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
        c[2] = cp_z = cos(delta_min + (gamma)/2);
    }else
    {
        c[0] = cp_x = sin(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
        c[1] = cp_y = cos(delta_min + (gamma)/2);
        c[2] = cp_z = cos(theta_min + (alpha)/2)*sin(delta_min + (gamma)/2);
    }

    modx = sqrt(cp_y*cp_y + cp_x*cp_x);
    mody = sqrt((cp_x*cp_z)*(cp_x*cp_z) + (cp_y*cp_z)*(cp_y*cp_z) + (cp_y*cp_y + cp_x*cp_x)*(cp_y*cp_y + cp_x*cp_x));

    for(int i = 0 ; i < V_res; i++)
    {
        y_ = -tan(gamma/2) + i*2*tan(gamma/2)/V_res;

        for(int j = 0; j < H_res; j++)
        {
            x_ = -tan(alpha/2) + j*2*tan(alpha/2)/H_res;

            x = cp_y*x_/modx + cp_x*cp_z*y_/mody + cp_x;
            y = -cp_x*x_/modx + cp_y*cp_z*y_/mody + cp_y;
            z = -(cp_y*cp_y + cp_x*cp_x)*y_/mody + cp_z;

            planer_coords[0] = x/sqrt(pow(x,2) + pow(y,2) + pow(z,2));
            planer_coords[1] = y/sqrt(pow(x,2) + pow(y,2) + pow(z,2));
            planer_coords[2] = z/sqrt(pow(x,2) + pow(y,2) + pow(z,2));

            world2cam(points2D, planer_coords, cam1_xc, cam1_yc, cam1_c, cam1_d, cam1_e, cam1_invpol, cam1_pol);

            ImgPointsx.at<float>(i,j) = points2D[0];
            ImgPointsy.at<float>(i,j) = points2D[1];
        }
    }

    cv::remap(M, img, ImgPointsx, ImgPointsy, 1);

    return img;
}

