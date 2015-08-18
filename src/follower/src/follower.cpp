#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <algorithm>
#include <cmath>

constexpr float PI = acos(-1);

ros::Publisher pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    auto it = std::min_element(msg->ranges.begin(), msg->ranges.end());
    auto minindex = std::distance(msg->ranges.begin(), it);

    float newangle = msg->angle_min + minindex * msg->angle_increment;
    float rightangle = newangle - PI/2;

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

    pub.publish(move);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "twoway");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("scan", 1, laserCallback);
    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);

    ros::spin();

    return EXIT_SUCCESS;
}
