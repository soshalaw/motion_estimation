#include "feature_detector.h"

feature_detector::feature_detector()
{
    //cv::ORB::create(nfeatures,scaleFactor,nlevel,edgeThreshold,firstLevel,WTA_K,scoreType = ORB::HARRIS_SCORE,patchSize,fastThreshold)
    orb = cv::ORB::create(15000,1.1f,10,20,2,2,cv::ORB::FAST_SCORE,20,20);

    matcher_br = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
}

void feature_detector::orb_descriptor(cv::Mat frame)
{
    orb->detectAndCompute(frame, cv::noArray(), kp2, desc2);

    matcher_br->match(desc1, desc2, matches);

    for (unsigned long i = 0; i < matches.size(); i++)
    {
        //std::cout << matches.at(i) << std::endl;
    }

    /*cv::drawKeypoints(img1, kp1, image_matches, cv::Scalar(0,255,0));

    cv::drawMatches(key_frame, kp_kfram, frame, kp2, matches, image_matches);

    cv::resize(image_matches, image_resize, cv::Size(3264/2,2448/4));

    cv::imshow("image", image_resize);

    int k = cv::waitKey(0);

    if (k%256 == 32)
    {
        std::string name = "/home/corelaptop02/Thesis/results/open_cv_img.png" ;
        cv::imwrite(name, image_resize);
    }

    cv::destroyWindow("image");*/
}

void feature_detector::set_keyframe(cv::Mat frame)
{
    frame.copyTo(key_frame);
    orb->detectAndCompute(key_frame, cv::noArray(), kp_kfram, desc1);
}
