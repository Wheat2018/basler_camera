//ROS头文件
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;

timeval t;
void photoCallback(const sensor_msgs::ImageConstPtr &msg)
{  
  timeval newT;
  gettimeofday(&newT, NULL);
  
  cout<<1000.0 / (1000.0 * (newT.tv_sec - t.tv_sec) + (newT.tv_usec - t.tv_usec) / 1000.0)<<"\t||| ";
  t = newT;
  ROS_INFO("Listener: recieved a photo of size %d * %d. The first pixel: %d",msg->width,msg->height,msg->data[0]);
  cv::Mat image;
  cv::cvtColor(cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::YUV422)->image,image,cv::COLOR_YUV2BGR_Y422);
  cv::imshow("view",image);
}

int main(int argc, char **argv)
{
  cv::namedWindow("view");
  cv::startWindowThread();
  gettimeofday(&t, NULL);
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("basler_acA1600_photo", 1, photoCallback);
  //ros::spin()用于调用所有可触发的回调函数。将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
  ros::spin(); 
  return 0;
}

