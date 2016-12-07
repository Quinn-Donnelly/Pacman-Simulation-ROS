#ifndef GHOST_ARBITER_H
#define GHOST_ARBITER_H

#include <vector>
#include <queue>
#include "ros/ros.h"
#include "package1/behavior.h"
#include "geometry_msgs/Twist.h"

#define BH_ARBITER_RATE 10
#define PRIORITY_WANDER 0
#define PRIORITY_AVOID 1

//comparison function for priority queue
//function is outside of class because it was simpler
//in order to be inside of the class, there had to be workarounds
//that made it not worth the trouble for this simple use
bool compare_priorities(std::pair<int, package1::behavior> a,
             std::pair<int, package1::behavior> b) {
    return a.first < b.first;
}

class Arbiter {
  public:
    
    Arbiter();
    void process_behaviors();

  private:
    ros::NodeHandle nh;

    //Priority queue for hold behavior messages
    std::priority_queue<std::pair<int, package1::behavior>,
            std::vector<std::pair<int, package1::behavior>>,
            decltype(&compare_priorities)> behavior_queue{compare_priorities};

    //Subscribers to behaviors, one callback for every behavior that is added
    ros::Subscriber sub_bh_wander;
    ros::Subscriber sub_bh_avoid;
    //Publisher to cmd_vel to move the robot
    ros::Publisher pub_vel;

    //Behavior Callbacks
    void cb_bh_wander(const package1::behavior::ConstPtr& msg);
    void cb_bh_avoid(const package1::behavior::ConstPtr& msg);

    //Robot movement
    void move_robot(package1::behavior& msg);
    void stop_robot();
};

#endif