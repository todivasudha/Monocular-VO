#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <opencv2/video/tracking.hpp>
#include "fast_detector.cpp"

void kltTracker(cv::Mat prevImg, cv::Mat nextImg, std::vector<cv::Point2f> &prevPts)
{
   std::vector<cv::Point2f> nextPts = prevPts;
   std::vector<uchar> status;
   std::vector<float> err;
   cv::calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts, status, err);
   for (int i = 0; i < prevPts.size(); ++i)
   {
   	 if(status[i])
   	 {
   	 	cv::line(prevImg, prevPts[i], nextPts[i], cv::Scalar(255));
   	 }
   }
   cv::namedWindow("window2", CV_WINDOW_AUTOSIZE);
   cv::imshow("window2", prevImg);
   cv::waitKey(0);
}

int main()
{
    /*cv::VideoCapture cap("/home/vasudha/monocular VO/video/fi-bu-m2.avi");
    cv::Mat prevImg, nextImg;
    cap>>prevImg;
    for (int i = 0; i < 5; ++i)
    {
      cap>>nextImg;
    }*/
  cv::Mat prevImg = cv::imread("/home/vasudha/monocular VO/images/9.jpg", CV_LOAD_IMAGE_COLOR);
  cv::Mat nextImg = cv::imread("/home/vasudha/monocular VO/images/10.jpg", CV_LOAD_IMAGE_COLOR);
  std::vector<cv::KeyPoint> prevKeypoints = fastDetector(prevImg);
  std::vector<cv::Point2f> prevPts;
  for (int i = 0; i < prevKeypoints.size(); ++i)
  {
  	prevPts.push_back(prevKeypoints[i].pt);
  }
  kltTracker(prevImg, nextImg, prevPts);
  return 0;
}