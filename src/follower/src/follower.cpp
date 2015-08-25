#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <algorithm>
#include <cmath>

constexpr float PI = acos(-1);

ros::Publisher pub;

template <class ForwardIterator>
ForwardIterator min_element_fuck_nans(ForwardIterator first, ForwardIterator last) {
    if (first == last) {
        return last;
    }
    ForwardIterator smallest = first;
    while (smallest != last && *smallest != *smallest) {
        ++smallest;
    }
    while (++first != last) {
        if (*first < *last && *first == *first) {
            smallest = first;
        }
    }
    return smallest;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    auto it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    if (it == msg->ranges.end()) {
        ROS_INFO("No data received.");
        return;
    }
    auto minindex = std::distance(msg->ranges.begin(), it);

    float newangle = msg->angle_min + minindex * msg->angle_increment;
    float rightangle = newangle - PI/2;
    
    ROS_INFO("min=%lf, minindex=%ld, rightangle=%lf", *it, minindex, rightangle);
    geometry_msgs::Twist move;

    if (rightangle > PI/2 || rightangle < -PI/2) {
        move.linear.x = 0;
    } else {
        move.linear.x = 0.1;
    }

    move.linear.y = 0;
    move.linear.z = 0;

    move.angular.x = 0;
    move.angular.y = 0;
    move.angular.z = rightangle;

    ROS_INFO("moving x=%lf, z=%lf", move.linear.x, move.angular.z);
    pub.publish(move);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twoway");
    ros::NodeHandle n;
    
    ROS_INFO("%lf", PI);

    ros::Subscriber sub = n.subscribe("scan", 1000, laserCallback);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
    
    ROS_INFO("setup successfully");
    ros::spin();

    return EXIT_SUCCESS;
}
