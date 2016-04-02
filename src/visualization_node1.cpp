#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include "opencv2/calib3d/calib3d.hpp"
#include <geometry_msgs/Point.h>

class Listener
{
public:
  ros::Publisher pub;
  void Callback(const cv_bridge::CvImage &message);
};


void Listener::Callback(const cv_bridge::CvImage &message)
{
	cv::Mat img = message.image;
	cv::Mat R = (cv::Mat_<double>(3,3) << 0,0,1,1,0,0,0,1,0);
	cv::Mat t = img.col(3);
	cv::Mat r(1, 3, CV_64F);
	cv::Rodrigues(R, r);
	std::cout<<"\n"<<r;
	nav_msgs::Odometry odom;
	odom.header.frame_id = "map";
	odom.header.stamp = ros::Time();
	odom.child_frame_id = "map";
	odom.pose.pose.position.x = t.at<double>(2,0);
	odom.pose.pose.position.y = t.at<double>(1,0);
	odom.pose.pose.position.z = t.at<double>(0,0);
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
	pub.publish(odom);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization_node1");
  Listener l;
  ros::NodeHandle n;
  l.pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
  ros::Subscriber sub = n.subscribe("camera_pose", 1000, &Listener::Callback, &l);
  ros::spin();
  return 0;
}