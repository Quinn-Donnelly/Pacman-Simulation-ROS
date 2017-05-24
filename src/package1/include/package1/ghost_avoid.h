#ifndef GHOST_AVOID_H
#define GHOST_AVOID_H

#define BH_AVOID_RATE 30
#define AVOID_VELOCITY 1

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "package1/behavior.h"

class Behavior_Avoid {

public:
    Behavior_Avoid();
    void process_behavior();
    void left_cb_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
	void right_cb_laser(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
    ros::NodeHandle nh;
    bool left_path_blocked = false;
	bool right_path_blocked = false;
	double left_laser_value;
	double right_laser_value;

    ros::Publisher pub_arbiter;
    ros::Subscriber sub_left_laser;
    ros::Subscriber sub_right_laser;

};

#endif