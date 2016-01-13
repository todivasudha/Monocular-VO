#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>

std::vector<cv::KeyPoint> fastDetector(cv::Mat image, int threshold = 100)
{
  cv::Mat img;
  cvtColor(image, img, CV_BGR2GRAY);
  std::vector<cv::KeyPoint> keypoints;
  cv::FAST(img, keypoints, threshold);
  return keypoints;
}

/*int main()
{
  cv::Mat image = cv::imread("/home/vasudha/monocular VO/images/9.jpg", CV_LOAD_IMAGE_COLOR);
  std::vector<cv::KeyPoint> keypoints;
  int threshold = 0;
  keypoints = fastDetector(image);
  cv::namedWindow("window", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar( "Threshold", "window", &threshold, 255);

  while(1)
  {  
   cv::Mat img = image.clone();  
   keypoints = fastDetector(img, threshold);
   std::cout<<keypoints.size()<<"\n";
   for (int i = 0; i < keypoints.size(); ++i)
   {
   	cv::circle(img, keypoints[i].pt, 5.0, cv::Scalar(0));
   }
   cv::imshow("window", img);
   int key = cv::waitKey(1);
   if(key == 27)
   	break;
 }
 cv::waitKey(0);
}*/
