#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle n;

    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;

    while (n.ok()) {
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0.3), tf::Vector3(0.1, 0.1, 0.06)),
                        ros::Time::now(), "base_link", "irobot_l_dist"));
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, -0.3), tf::Vector3(0.1, -0.1, 0.06)),
                        ros::Time::now(), "base_link", "irobot_r_dist"));
        broadcaster.sendTransform(
                tf::StampedTransform(
                        tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(0, 0, 0)),
                        ros::Time::now(), "base_link", "irobot_gps_controller"));

        r.sleep();
    }
}