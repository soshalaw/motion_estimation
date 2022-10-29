#include "image_bridge.h"

image_bridge::image_bridge()
: it_(nh_)
{
    image_sub1 = it_.subscribe("usb_cam1/image", 1 , &image_bridge::imageCbA, this);
    image_sub2 = it_.subscribe("usb_cam2/image", 1 , &image_bridge::imageCbB, this);
    cv::namedWindow(OPENCV_WINDOW1);

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

    ros::param::get("delta_min", delta_min);
    ros::param::get("delta_max", delta_max);

    ros::param::get("theta_min", theta_min);
    ros::param::get("theta_max", theta_max);

    ros::param::get("alpha_min", alpha_min);
    ros::param::get("alpha_max", alpha_max);

    ros::param::get("H_res", H_res); // horizontal length of the output image
}

image_bridge::~image_bridge()
{
     cv::destroyWindow(OPENCV_WINDOW1);
}

void image_bridge::imageCbA(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exceprion: %s",e.what());
        return;
    }

    img1 = cv_ptr->image;
    out_img.header = cv_ptr->header;
    out_img.encoding = cv_ptr->encoding;
    out_img.image = img1;

    image_pub.publish(out_img.toImageMsg());
}


void image_bridge::imageCbB(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exceprion: %s",e.what());
        return;
    }

    img2 = cv_ptr->image;

    cuboidal_proj();
    //panorama();
}

void image_bridge::cuboidal_proj() // cuboidal projection of the 360 image view
{
    //double theta1_min, theta1_max, theta2_min, theta2_max, theta3_min, theta3_max, theta4_min, theta4_max, theta5_min, theta5_max, theta6_min, theta6_max, theta7_min, theta7_max, theta8_min, theta8_max;
    //double alpha1_min, alpha1_max, alpha2_min, alpha2_max, alpha3_min, alpha3_max, alpha4_min, alpha4_max, alpha5_min, alpha5_max, alpha6_min, alpha6_max, alpha7_min, alpha7_max, alpha8_min, alpha8_max;

    new_img = cv::Mat::zeros(int(3*H_res),int(3*H_res),img1.type());
    proj_cols = 0;
    proj_rows = 0;

    for (int i = 0; i < 3 ; i++)
    {
       proj = image_proc.slice(img1, c, (theta_min+90*i), (theta_max+90*i), alpha_min, alpha_max, cam1_xc, cam1_yc, cam1_c, cam1_d, cam1_e, cam1_invpol);

       proj.copyTo(new_img(cv::Rect(proj_cols,proj.rows,proj.cols,proj.rows)));

       if(i == 1)
       {
         for (int j = 0; j < 3; j++)
         {
             proj = image_proc.slice(img1,c,(theta_min+90*i),(theta_max+90*i),(135 - 90*j),(225 - 90*j),cam1_xc, cam1_yc, cam1_c, cam1_d, cam1_e, cam1_invpol);

             proj.copyTo(new_img(cv::Rect(proj_cols,proj_rows,proj.cols,proj.rows)));

             proj_rows = proj_rows + proj.rows;
         }
       }

       proj_cols = proj_cols + proj.cols;
    }
    cv::imshow(OPENCV_WINDOW1, new_img);

    int k = cv::waitKey(1);

    if (k%256 == 32)
    {
        //std::string name = "/home/corelaptop02/internship/camera_calibration/camera_07/open_cv_img" + std::to_string(img_counter) + ".png" ;
        std::string name = "/home/corelaptop02/Thesis/samples/dual_cam/dual_cam_cuboid.png";
        cv::imwrite(name, new_img);
    }
}

void image_bridge::panorama() //merging the images of the two cameras into one matrix
{
    // create panoramic images from images of camera 1 & 2
    pan_1 = image_proc.panaroma(img1, delta_min, delta_max, cam1_xc, cam1_yc, cam1_c, cam1_d, cam1_e, cam1_invpol);
    pan_2 = image_proc.panaroma(img2, delta_min, delta_max, cam2_xc, cam2_yc, cam2_c, cam2_d, cam2_e, cam2_invpol);

    //create new image matrix of size (img1_rows,2*img1_cols) assuming both images are of same size
    
    new_img = cv::Mat::zeros(pan_1.rows, (pan_1.cols+pan_2.cols), img1.type());
  
    // copy the image pair to the image matrix
    pan_1.copyTo(new_img(cv::Rect(0,0,pan_1.cols,pan_1.rows)));
    pan_2.copyTo(new_img(cv::Rect(pan_1.cols,0,pan_2.cols,pan_2.rows)));;

    //resize the image to fit the monitor screen for visualization

    cv::resize(new_img, res, cv::Size(new_img.cols*3, new_img.rows*3));

    cv::imshow(OPENCV_WINDOW1, res);

    int k = cv::waitKey(1);

    if (k%256 == 32)
    {
        //std::string name = "/home/corelaptop02/internship/camera_calibration/camera_07/open_cv_img" + std::to_string(img_counter) + ".png" ;
        std::string name = "/home/corelaptop02/Thesis/samples/dual_cam/dual_cam1_panorama.png";
        cv::imwrite(name, new_img);
    }
}
