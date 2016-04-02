#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
  int frame_id = 0;
  ros::init(argc, argv, "dataset_node");
  cv::VideoCapture cap("/home/vasudha/catkin_ws/src/monocular_vo/video/kitti/2011_09_26_drive_0009_extract/image_03/data/%10d.png");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher pub = n.advertise<cv_bridge::CvImage>("logitech_camera1/image", 1000);
  cv_bridge::CvImage message;
  message.header.stamp = ros::Time::now();
  message.encoding = sensor_msgs::image_encodings::BGR8;
  while( cap.isOpened() )
  {
    	cv::Mat img;
    	cap.read(img);
      message.header.seq = frame_id;
      message.header.frame_id = frame_id;
      message.image = img;
    	pub.publish(message);
    	loop_rate.sleep();
      frame_id++;
  }
  return 0;
}