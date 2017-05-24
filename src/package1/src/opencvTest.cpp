#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <opencv/cv.h>
#include "package1/behavior.h"

ros::Publisher pub_arbiter;

void publishOffset(int offset, bool active) {
  package1::behavior msg;

  msg.active = active;
  msg.vel_turn = offset;

  pub_arbiter.publish(msg);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat hsv_image;
    cv::cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, hsv_image, cv::COLOR_BGR2HSV);

    // Threshold the HSV image, keep only the red pixels
    cv::Mat binaryImage;
    cv::inRange(hsv_image, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), binaryImage);

    cv::Mat locations;   // output, locations of non-zero pixels
    cv::findNonZero(binaryImage, locations);
    // access pixel coordinates
    for(int i = 0; i < locations.total(); ++i){
      std::cout << "x = " << locations.at<cv::Point>(i).x << " y = " << locations.at<cv::Point>(i).y << std::endl;
      publishOffset(locations.at<cv::Point>(i).x, true);
    }

    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imshow("view", binaryImage);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("irobot/camera/image_raw", 1, imageCallback);
  ros::Rate loop_rate(30);
  pub_arbiter = nh.advertise<package1::behavior>("behavior/offset", 1);

  while(ros::ok()) {
    cv::startWindowThread();
    ros::spinOnce();
    loop_rate.sleep();
  }

  cv::destroyWindow("view");
}