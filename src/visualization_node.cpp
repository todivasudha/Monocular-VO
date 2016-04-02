#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
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
	//cv::Mat R = img(cv::Rect(0,0,3,3));
	cv::Mat R = (cv::Mat_<double>(3,3) << 0,0,1,1,0,0,0,1,0);
	cv::Mat t = img.col(3);
	cv::Mat r(1, 3, CV_64F);
	cv::Rodrigues(R, r);
	std::cout<<"\n"<<r;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = t.at<double>(0,0);
	marker.pose.position.y = t.at<double>(1,0);
	marker.pose.position.z = t.at<double>(2,0);
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	pub.publish(marker);
	/*visualization_msgs::Marker point, line_strip, marker;
	point.header.frame_id = line_strip.header.frame_id = "map";
	point.header.stamp = line_strip.header.stamp = ros::Time();
	point.ns = line_strip.ns = "my_namespace";
	point.id = 0;
	line_strip.id = 1;
	point.type = visualization_msgs::Marker::POINTS;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	point.action = line_strip.action = visualization_msgs::Marker::ADD;
	point.frame_locked = line_strip.frame_locked = 0;
	geometry_msgs::Point p;
    p.x = t.at<double>(0,0);
    p.y = t.at<double>(1,0);
    p.z = t.at<double>(2,0);
    point.points.push_back(p);
    line_strip.points.push_back(p);
	point.scale.x = 0.2;
    point.scale.y = 0.2;
	line_strip.scale.x = 0.1;
	point.color.a = line_strip.color.a = 1.0;
	point.color.g = 1.0;
	line_strip.color.b = 1.0;
	pub.publish(point);
	pub.publish(line_strip);*/
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visualization_node");
  Listener l;
  ros::NodeHandle n;
  l.pub = n.advertise<visualization_msgs::Marker>("marker", 1000);
  ros::Subscriber sub = n.subscribe("camera_pose", 1000, &Listener::Callback, &l);
  ros::spin();
  return 0;
}