#include "odom.h"

namespace odom {

    /*R(cv::Mat::eye(3, 3, CV_64FC1)),
    t(cv::Mat::zeros(3, 1, CV_64FC1)),
    camera_pose(cv::Mat::eye(3, 4, CV_64FC1))*/
    Frame::Frame()
    {}

    /*R(cv::Mat::eye(3, 3, CV_64FC1)),
    t(cv::Mat::zeros(3, 1, CV_64FC1)),
    camera_pose(cv::Mat::eye(3, 4, CV_64FC1))*/
    Frame::Frame(cv::Mat image_)
    {
        cv::cvtColor(image_, image_gray, cv::COLOR_BGR2GRAY);
    }

    feature_detector::feature_detector()
    {
        //cv::ORB::create(nfeatures,scaleFactor,nlevel,edgeThreshold,firstLevel,WTA_K,scoreType = ORB::HARRIS_SCORE,patchSize,fastThreshold)
        orb = cv::ORB::create(20000,1.1f,10,20,2,2,cv::ORB::FAST_SCORE,20,20);
        //matcher = cv::BFMatcher(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    }

    void feature_detector::set_keyframe(cv::Mat frame)
    {
        frame.copyTo(key_frame);

        int cols = key_frame.cols;
        int rows = key_frame.rows;

        orb->detect(key_frame, keypoints_keyframe_, cv::noArray());

        keypoints_keyframe = ssc(keypoints_keyframe_, 1000, 0.1f ,cols, rows);

        cv::KeyPoint::convert(keypoints_keyframe, points_keyframe);

    }

    void feature_detector::estimate_opticalflw(cv::Mat frame2)
    {
        /*orb->detect(frame1, keypoints_keyframe_, cv::noArray());

        keypoints_keyframe = ssc(keypoints_keyframe_, 1000, 0.1f ,cols, rows);

        cv::KeyPoint::convert(keypoints_keyframe, points_keyframe);

        orb->detect(frame2, keypoints_frame_, cv::noArray());

        keypoints_frame = ssc(keypoints_frame_, 1000, 0.1f ,cols, rows);

        cv::KeyPoint::convert(keypoints_frame, points_keyframe);*/

        std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 30, 0.01);

        cv::calcOpticalFlowPyrLK(key_frame, frame2, points_keyframe, points_frame, status, err, cv::Size(15,15), 2, criteria);
        int indexCorrection = 0;
        for (unsigned long i = 0; i < points_frame.size();i ++)
        {
            cv::Point2d pt = points_frame.at(i - indexCorrection);
            if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))
            {
                if((pt.x<0)||(pt.y<0)) {
                    status.at(i) = 0;
                }
                points_keyframe.erase (points_keyframe.begin() + (i - indexCorrection));
                points_frame.erase (points_frame.begin() + (i - indexCorrection));
                indexCorrection++;
             }
        }

        E = cv::findEssentialMat(points_keyframe, points_frame,camera_matrix,cv::RANSAC, 0.999, 1, mask);

        goodMatchesFeatures(points_keyframe, points_frame, mask);

        /*if (points_frame.size() < 200)
        {
            std::cout << "Not enough features to track: " << points_keyframe.size() << std::endl;
        }*/
         std::cout << points_frame.size() << std::endl;

        cv::Mat frame;

        cv::cvtColor(frame2, frame, cv::COLOR_GRAY2BGR);
        for (unsigned long i = 0; i < points_frame.size(); i++)
        {
            cv::circle(frame, points_frame.at(i), 1, cv::Scalar(0,0,255),-1);
            //cv::circle(frame2, points_keyframe.at(i), 1, cv::Scalar(255,0,0),-1);
            //cv::line(frame2, points_keyframe.at(i), points_frame.at(i),cv::Scalar(255,0,255));

        }

        cv::imshow("img", frame);
        cv::waitKey(1);

        //cv::recoverPose(E, points_keyframe, points_frame, camera_matrix, R, t, mask);
    }

    void feature_detector::goodMatchesFeatures(std::vector<cv::Point2f>& points1, std::vector<cv::Point2f>& points2, cv::Mat mask)
    {
        std::vector<cv::Point2f> inlier_match_points1, inlier_match_points2;
        for(int i = 0; i < mask.rows; i++)
        {
            if(mask.at<unsigned char>(i))
            {
                inlier_match_points1.push_back(cv::Point2f( points1.at(i).x, points1.at(i).y));
                inlier_match_points2.push_back(cv::Point2f( points2.at(i).x, points2.at(i).y));
            }
        }
        points1 = inlier_match_points1;
        points2 = inlier_match_points2;
    }


    void feature_detector::estimate_matching (cv::Mat frame1, cv::Mat frame2)
    {
        int cols = frame1.cols;
        int rows = frame1.rows;

        orb->detect(frame1, keypoints_keyframe_, cv::noArray());

        keypoints_keyframe = ssc(keypoints_keyframe_, 1000, 0.1f ,cols, rows);

        orb->compute(frame1, keypoints_keyframe, desc1);

        orb->detect(frame2, keypoints_frame_, cv::noArray());

        keypoints_frame = ssc(keypoints_frame_, 1000, 0.1f ,cols, rows);

        orb->compute(frame2, keypoints_frame, desc2);

        matcher.knnMatch(desc1, desc2, matches, 2);

        std::vector<cv::DMatch> good_matches;

        for (unsigned long i = 0; i < matches.size(); i++) {

            if(matches[i][0].distance  < 0.7f*matches[i][1].distance){

               points_keyframe.push_back(keypoints_keyframe[matches[i][0].trainIdx].pt);
               points_frame.push_back(keypoints_frame[matches[i][0].queryIdx].pt);

               //good_matches.push_back(matches[i][0]);
            }
        }

        //cv::Mat goodmatches;

        //std::cout << good_matches.size() << std::endl;
        //cv::drawMatches(frame1, keypoints_keyframe, frame2, keypoints_frame, good_matches, goodmatches);
        //cv::imshow("good matches", goodmatches);

        E = cv::findEssentialMat(points_keyframe, points_frame,camera_matrix,cv::RANSAC, 0.999, 1, mask);
        
        cv::recoverPose(E, points_keyframe, points_frame, camera_matrix, R, t, mask);

        //cv::recoverPose()
        
        good_matches.clear();
        matches.clear();
        points_frame.clear();
        points_keyframe.clear();
    }


std::vector<cv::KeyPoint> feature_detector::ssc(std::vector<cv::KeyPoint> keyPoints, int numRetPoints,float tolerance, int cols, int rows)
{
    //sort the keypoints

    for (unsigned int i = 0; i < keyPoints.size(); i++)
    {
        responseVector.push_back(keyPoints[i].response);   
    }

    std::vector<int> Indx(responseVector.size());
    std::iota(std::begin(Indx), std::end(Indx), 0);

    #if CV_MAJOR_VERSION >= 4
      cv::sortIdx(responseVector, Indx, cv::SORT_DESCENDING);
    #else
      cv::sortIdx(responseVector, Indx, CV_SORT_DESCENDING);
    #endif

    for (unsigned int i = 0; i < keyPoints.size(); i++)
    {
        keyPointsSorted.push_back(keyPoints[Indx[i]]);
    }

    // several temp expression variables to simplify solution equation
    int exp1 = rows + cols + 2 * numRetPoints;
    long long exp2 =
      ((long long)4 * cols + (long long)4 * numRetPoints +
       (long long)4 * rows * numRetPoints + (long long)rows * rows +
       (long long)cols * cols - (long long)2 * rows * cols +
       (long long)4 * rows * cols * numRetPoints);
    double exp3 = sqrt(exp2);
    double exp4 = numRetPoints - 1;

    double sol1 = -round((exp1 + exp3) / exp4); // first solution
    double sol2 = -round((exp1 - exp3) / exp4); // second solution

    // binary search range initialization with positive solution
    int high = (sol1 > sol2) ? sol1 : sol2;
    int low = floor(sqrt((double)keyPointsSorted.size() / numRetPoints));
    low = std::max(1, low);

    int width;
    int prevWidth = -1;

    std::vector<int> ResultVec;
    bool complete = false;
    unsigned int K = numRetPoints;
    unsigned int Kmin = round(K - (K * tolerance));
    unsigned int Kmax = round(K + (K * tolerance));

    std::vector<int> result;
    result.reserve(keyPointsSorted.size());
    while (!complete) {
    width = low + (high - low) / 2;
    if (width == prevWidth ||
        low >
            high) { // needed to reassure the same radius is not repeated again
      ResultVec = result; // return the keypoints from the previous iteration
      break;
    }
    result.clear();
    double c = (double)width / 2.0; // initializing Grid
    int numCellCols = floor(cols / c);
    int numCellRows = floor(rows / c);
    std::vector<std::vector<bool>> coveredVec(numCellRows + 1,
                                    std::vector<bool>(numCellCols + 1, false));

    for (unsigned int i = 0; i < keyPointsSorted.size(); ++i) {
      int row =
          floor(keyPointsSorted[i].pt.y /
                c); // get position of the cell current point is located at
      int col = floor(keyPointsSorted[i].pt.x / c);
      if (coveredVec[row][col] == false) { // if the cell is not covered
        result.push_back(i);
        int rowMin = ((row - floor(width / c)) >= 0)
                         ? (row - floor(width / c))
                         : 0; // get range which current radius is covering
        int rowMax = ((row + floor(width / c)) <= numCellRows)
                         ? (row + floor(width / c))
                         : numCellRows;
        int colMin =
            ((col - floor(width / c)) >= 0) ? (col - floor(width / c)) : 0;
        int colMax = ((col + floor(width / c)) <= numCellCols)
                         ? (col + floor(width / c))
                         : numCellCols;
        for (int rowToCov = rowMin; rowToCov <= rowMax; ++rowToCov) {
          for (int colToCov = colMin; colToCov <= colMax; ++colToCov) {
            if (!coveredVec[rowToCov][colToCov])
              coveredVec[rowToCov][colToCov] =
                  true; // cover cells within the square bounding box with width
                        // w
          }
        }
      }
    }

    if (result.size() >= Kmin && result.size() <= Kmax) { // solution found
      ResultVec = result;
      complete = true;
    } else if (result.size() < Kmin)
      high = width - 1; // update binary search range
    else
      low = width + 1;
    prevWidth = width;
    }
    // retrieve final keypoints
    std::vector<cv::KeyPoint> kp;
    for (unsigned int i = 0; i < ResultVec.size(); i++)
    kp.push_back(keyPointsSorted[ResultVec[i]]);

    responseVector.clear();
    keyPointsSorted.clear();

    return kp;
}

}
