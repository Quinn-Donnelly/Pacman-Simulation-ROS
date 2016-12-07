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
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::inRange(hsv_image, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), lower_red_hue_range);
    bool whiteFound = false;
    int first = 0;
    int last = lower_red_hue_range.cols;

    for(int y = 0; y < lower_red_hue_range.rows; y++)
    {
      for(int x = 0; x < lower_red_hue_range.cols; x++)
      {
          if (lower_red_hue_range.at<cv::Vec3b>(x,y)[0] == 255 && lower_red_hue_range.at<cv::Vec3b>(x,y)[1] == 255 && lower_red_hue_range.at<cv::Vec3b>(x,y)[2] == 255)
          {
            if(!whiteFound){
              first = x;
              whiteFound = true;
            } else {
              last = x;
            }
          }
      }
      if(whiteFound){break;}
    }

    std::cout << "first = " << first << " last = " << last << std::endl;

    if(!whiteFound)
    {
      publishOffset(0, false);
    } else{
      publishOffset((last + first ) / 2 - lower_red_hue_range.cols / 2, true);
    }
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imshow("view", lower_red_hue_range);
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

  pub_arbiter = nh.advertise<package1::behavior>("behavior/offset", 1);

  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  std::string node_name = argv[1];
  image_transport::Subscriber sub = it.subscribe("/" + node_name + "/irobot/camera/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}