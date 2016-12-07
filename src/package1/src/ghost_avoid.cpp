#include "ghost_avoid.h"

Behavior_Avoid::Behavior_Avoid() {
	this->sub_left_laser = this->nh.subscribe("irobot/left_distance_scan", 1, 
												&Behavior_Avoid::left_cb_laser, this);
	this->sub_right_laser = this->nh.subscribe("irobot/right_distance_scan", 1, 
												&Behavior_Avoid::right_cb_laser, this);
	this->pub_arbiter = this->nh.advertise<package1::behavior>("behavior/avoid", 1);
}

void Behavior_Avoid::left_cb_laser (const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Angle Min: %f Max: %f Inc: %f", msg->angle_min, msg->angle_max, 
    														msg->angle_increment);
    ROS_INFO("Angle Min: %f Max: %f", msg->range_min, msg->range_max);
    for (int i = 0; i < msg->ranges.size(); i++){
        if (msg->ranges[i] < 1.10) {
            this->left_path_blocked = true;
            this->left_laser_value = msg->ranges[i];
        }
    }
}
 
void Behavior_Avoid::right_cb_laser (const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Angle Min: %f Max: %f Inc: %f", msg->angle_min, msg->angle_max,
    														 msg->angle_increment);
    ROS_INFO("Angle Min: %f Max: %f", msg->range_min, msg->range_max);
    for (int i = 0; i < msg->ranges.size(); i++){
        if (msg->ranges[i] < 1.10) {
            this->right_path_blocked = true;
            this->right_laser_value = msg->ranges[i];
        }
    }
}

void Behavior_Avoid::process_behavior() {
	package1::behavior msg;
    
    if(this->left_path_blocked && this->right_path_blocked) {
        msg.active = true;
        if(this->left_laser_value < this->right_laser_value) {
            msg.vel_fw = -AVOID_VELOCITY;
            msg.vel_turn = -AVOID_VELOCITY;
        } else {
            msg.vel_fw = -AVOID_VELOCITY;
            msg.vel_turn = AVOID_VELOCITY;
        }
    } else if(this->left_path_blocked) {
        msg.active = true;
        msg.vel_turn = -AVOID_VELOCITY;
    } else if (this->right_path_blocked) {
        msg.active = true;
        msg.vel_turn = AVOID_VELOCITY;
    } else if (this->left_path_blocked && this->right_path_blocked) {
        msg.active = true;
        if(this->left_laser_value < this->right_laser_value) {
            msg.vel_fw = -AVOID_VELOCITY;
            msg.vel_turn = -AVOID_VELOCITY;
        } else {
            msg.vel_fw = -AVOID_VELOCITY;
            msg.vel_turn = AVOID_VELOCITY;
        }
    } else {
        msg.active = false;
        msg.vel_turn = 0;
    }

    this->pub_arbiter.publish(msg);
    this->left_path_blocked = false;
	this->right_path_blocked = false;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ghost_avoid");
	ros::NodeHandle nh;
	ROS_INFO("Starting ghost avoid node...");
	
	Behavior_Avoid behavior_avoid;
	ros::Rate loop_rate(BH_AVOID_RATE);

    while(ros::ok()) {
	    behavior_avoid.process_behavior();

	    ros::spinOnce();
	    loop_rate.sleep();
    }

	return 0;
}