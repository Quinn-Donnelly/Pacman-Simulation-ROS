#ifndef CLASS_HOMEWORK_WANDER_H_H
#define CLASS_HOMEWORK_WANDER_H_H

#define BH_WANDER_RATE 30
#define WANDER_VELOCITY 2

#include "ros/ros.h"
#include <termios.h>
#include "wasd/behavior.h"

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