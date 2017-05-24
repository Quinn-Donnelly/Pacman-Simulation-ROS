#include "ghost_arbiter.h"

Arbiter::Arbiter() {
    //Set subscriptions & advertisement
    this->sub_bh_wander = this->nh.subscribe("behavior/wander", 1, &Arbiter::cb_bh_wander, this);
    this->sub_bh_avoid = this->nh.subscribe("behavior/avoid", 1, &Arbiter::cb_bh_avoid, this);
    this->sub_bh_offset = this->nh.subscribe("behavior/offset", 1, &Arbiter::cb_bh_offset, this);
    this->pub_vel = this->nh.advertise<geometry_msgs::Twist>("irobot/cmd_vel", 1);
}

void Arbiter::process_behaviors() {
    if (!this->behavior_queue.empty()) {
        package1::behavior priority_msg = this->behavior_queue.top().second;
        move_robot(priority_msg);
        while(this->behavior_queue.size() > 0) {
            //C++ Priority Queue has no clear function,
            //so we loop to pop off the old messages
            this->behavior_queue.pop();
        }
    }
    else {
        stop_robot();
    }
}

void Arbiter::cb_bh_offset(const package1::behavior::ConstPtr &msg){
    if(msg->active){
        
        package1::behavior new_msg;
        int offset = msg->vel_turn;
        if(offset > 400){
            new_msg.vel_turn = -0.5;
        } else if (offset <= 400) {
            new_msg.vel_turn = 0.5;
        }
        new_msg.vel_fw = 1;
        this->behavior_queue.push(std::pair<int, package1::behavior>(PRIORITY_OFFSET, new_msg));
    }
}

void Arbiter::cb_bh_avoid(const package1::behavior::ConstPtr &msg) {
    if (msg->active) {
        this->behavior_queue.push(std::pair<int, package1::behavior>(PRIORITY_AVOID, *msg));
    }
    ROS_DEBUG("Arbiter: Avoid(%s) Fw: %.1f Turn: %.1f", msg->active ? "on" : "off", msg->vel_fw, msg->vel_turn);
}

void Arbiter::cb_bh_wander(const package1::behavior::ConstPtr &msg) {
    if (msg->active) {
        this->behavior_queue.push(std::pair<int, package1::behavior>(PRIORITY_WANDER, *msg));
    }
    ROS_DEBUG("Arbiter: Wander(%s) Fw: %.1f Turn: %.1f", msg->active ? "on" : "off", msg->vel_fw, msg->vel_turn);
}

void Arbiter::move_robot(package1::behavior& msg) {
    geometry_msgs::Twist msg_move;
    msg_move.linear.x = msg.vel_fw;
    msg_move.angular.z = msg.vel_turn;
    this->pub_vel.publish(msg_move);
}

void Arbiter::stop_robot() {
    geometry_msgs::Twist msg_move;
    msg_move.linear.x = 0;
    msg_move.angular.z = 0;
    this->pub_vel.publish(msg_move);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bh_arbiter");
    ros::NodeHandle nh;
    ROS_INFO("Arbiter node started...");
    ros::Rate loop_rate(BH_ARBITER_RATE);

    //Create the arbiter
    Arbiter arbiter;

    while (ros::ok()) {
        arbiter.process_behaviors();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}