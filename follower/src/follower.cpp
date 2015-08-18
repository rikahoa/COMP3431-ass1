#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <algorithm>
#include <sstream>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (!msg->ranges.empty()) {

        auto min_distance = std::min_element(msg->ranges.begin(), msg->ranges.end());

        ROS_INFO("min range is %f", *min_distance);
    }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "follower");
    ros::NodeHandle n;
    
    ros::Subscriber sub = n.subscribe("scan", 1000, scanCallback);
    ROS_INFO("STARTING SUBSCRIBER BITCHES");

    ros::spin();

    return 0;
}
