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

int main()
{
	/*cv::VideoCapture cap("/home/vasudha/monocular VO/video/fi-bu-m2.avi");
    cv::Mat prevImg, nextImg;
    cap>>prevImg;
    for (int i = 0; i < 5; ++i)
    {
      cap>>nextImg;
    }*/
  cv::Mat prevImg = cv::imread("9.jpg", CV_LOAD_IMAGE_COLOR);
  cv::Mat nextImg = cv::imread("10.jpg", CV_LOAD_IMAGE_COLOR);
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
  cv::Mat F;
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
   std::cout<<"\nNo. of points : "<<srcPts.size();
   std::cout<<"\nNo. of inliers : "<<inlier;
   std::cout<<"\n";
   cv::namedWindow("window", CV_WINDOW_AUTOSIZE);
   cv::imshow("window", prevImg);
   std::cout<<"Fundamental matrix : \n";
   for (i = 0; i < 3; ++i)
   {
   	for (j = 0; j < 3; ++j)
   	{
  		std::cout<<F.at<double>(i,j)<<" ";
  	}
  	std::cout<<"\n";
   }
   cv::waitKey(0);
   return 0;
}