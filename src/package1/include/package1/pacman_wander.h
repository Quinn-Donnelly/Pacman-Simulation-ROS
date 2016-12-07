#ifndef PACMAN_WANDER_H
#define PACMAN_WANDER_H

#include "ros/ros.h"
#include <termios.h>
#include "package1/behavior.h"

#define BH_WANDER_RATE 30
#define WANDER_VELOCITY 2

class Behavior_Wander {

public:
    Behavior_Wander();
    void process_behavior();
    // non-blocking input function
    char getch();

private:
    ros::NodeHandle nh;

    //publishing to send velocity messages to the arbiter
    ros::Publisher pub_arbiter;

};

#endif