#ifndef CLASS_HOMEWORK_ARBITER_H
#define CLASS_HOMEWORK_ARBITER_H

#define BH_ARBITER_RATE 30

#include <vector>
#include <queue>
#include <functional>
#include "ros/ros.h"
#include "wasd/behavior.h"
#include "geometry_msgs/Twist.h"

#define PRIORITY_WANDER 0

//comparison function for priority queue
//function is outside of class because it was simpler
//in order to be inside of the class, there had to be workarounds
//that made it not worth the trouble for this simple use
bool compare_priorities(std::pair<int, wasd::behavior> a,           
                std::pair<int, wasd::behavior> b){
    return a.first < b.first;
}

class Arbiter {
  public:
    
    Arbiter();
    void process_behaviors();

  private:
    ros::NodeHandle nh;

    //Priority queue for hold behavior messages
    std::priority_queue<std::pair<int, wasd::behavior>,
            std::vector<std::pair<int, wasd::behavior>>,
            decltype(&compare_priorities)> behavior_queue{compare_priorities};

    //Subscribers to behaviors, one callback for every behavior that is added
    ros::Subscriber sub_bh_wander;
    //Publisher to cmd_vel to move the robot
    ros::Publisher pub_vel;

    //Behavior Callbacks
    void cb_bh_wander(const wasd::behavior::ConstPtr& msg);
    void cb_bh_avoid(const wasd::behavior::ConstPtr& msg);

    //Robot movement
    void move_robot(wasd::behavior& msg);
    void stop_robot();
};

#endif