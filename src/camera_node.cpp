#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "fundamental_mat.cpp"

class Listener
{
	
  cv::Mat prevImg;
  cv::Mat nextImg;
  cv::Mat C_n;
public:
  Listener()
  {
  	prevImg.data = nextImg.data = NULL;
  	C_n = cv::Mat::eye(4, 4, CV_64F);
  }
  ros::Publisher pub;
  void Callback(const cv_bridge::CvImage &message);
};


void Listener::Callback(const cv_bridge::CvImage &message)
{
  cv_bridge::CvImage img;
  nextImg = message.image;
  C_n = fundamentalMat(prevImg, nextImg, C_n);
  prevImg = nextImg;
  img.header.seq = message.header.seq;
  img.header.frame_id = message.header.frame_id;
  img.header.stamp = ros::Time::now();
  img.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
  img.image = C_n;
  pub.publish(img);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_node");
  /*cv::VideoCapture cap("/home/vasudha/monocular VO/video/fi-bu-m2.avi");
    cv::Mat prevImg, nextImg;
    cap>>prevImg;
    for (int i = 0; i < 5; ++i)
    {
      cap>>nextImg;
    }*/
  Listener l;
  ros::NodeHandle n;
  l.pub = n.advertise<cv_bridge::CvImage>("camera_pose", 1000);
  ros::Subscriber sub = n.subscribe("logitech_camera1/image", 1000, &Listener::Callback, &l);
  ros::spin();
  return 0;
}