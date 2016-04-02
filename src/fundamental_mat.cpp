#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/video/tracking.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "fast_detector.cpp"

cv::Mat fundamentalMat(cv::Mat prevImg, cv::Mat nextImg, cv::Mat C_n)
{
  if(!prevImg.data)
  {
    prevImg = nextImg;
    return C_n;
  }
  cv::Mat C;
  std::vector<cv::KeyPoint> prevKeypoints = fastDetector(prevImg);
  std::vector<cv::Point2f> prevPts;
  int i, j;
  for (i = 0; i < prevKeypoints.size(); ++i)
  {
  	prevPts.push_back(prevKeypoints[i].pt);
  }
  std::vector<cv::Point2f> nextPts = prevPts;
  std::vector<uchar> status;
  std::vector<float> err;
  cv::calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err);
  std::vector<cv::Point2f> srcPts, dstPts;
  int ransacReprojThreshold = 1;
  std::vector<uchar> mask;
  cv::Mat F, K, Kt, E;
  for (i = 0; i < prevPts.size(); i+=1)
  {
     if(status[i])
     {
     	srcPts.push_back(prevPts[i]);
     	dstPts.push_back(nextPts[i]);
     }
  }
  int inlier = 0;
  	F = cv::findFundamentalMat(srcPts, dstPts, CV_FM_RANSAC, 3, 0.99, mask);
   for (i = 0; i < srcPts.size() ; ++i)
   {
   	if(mask[i])
      {cv::line(prevImg, srcPts[i], dstPts[i], cv::Scalar(255));
       inlier++;
      }
   }
   //std::cout<<"\nNo. of points : "<<srcPts.size();
   //std::cout<<"\nNo. of inliers : "<<inlier;
   //std::cout<<"\n";
   cv::namedWindow("window", CV_WINDOW_AUTOSIZE);
   cv::imshow("window", prevImg);
   //std::cout<<"\nFundamental matrix : \n"<<F;

   K = (cv::Mat_<double>(3,3) << 9.037596e+02, 0.000000e+00, 6.957519e+02, 0.000000e+00, 9.019653e+02, 2.242509e+02, 0.000000e+00, 0.000000e+00, 1.000000e+00);
   cv::transpose(K, Kt);
   E = Kt*F*K;
   //std::cout<<"\nEssential matrix : \n"<<E;

   cv::SVD A = cv::SVD(E, cv::SVD::FULL_UV);
   cv::Mat W = (cv::Mat_<double>(3,3) << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
   cv::Mat R = (A.u)*W*(A.vt);
   cv::Mat t = A.u.col(2);
   //std::cout<<"\nTranslation matrix : \n"<<t;
   //std::cout<<"\nRotation matrix : \n"<<R;

   cv::hconcat(R, t, C);
   cv::Mat temp = (cv::Mat_<double>(1,4)<< 0, 0, 0, 1);
   cv::vconcat(C, temp, C);
   //std::cout<<"\nTrnformation : \n"<<C;
   C_n = C_n*C;
   //std::cout<<"\nIncremental pose : \n"<<C_n;
   cv::Mat t1 = C_n.col(3);
   std::cout<<"\n"<<t1;
   cv::waitKey(100);
   return C_n;  
}