#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "odom_clean");
    ros::NodeHandle n;

    ros::Publisher clean_odom_pub = n.advertise<nav_msgs::Odometry>("/ass1/odom", 1);
    
    // put transforms here

    return 0;
}
