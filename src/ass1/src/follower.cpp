#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/TwistStamped.h"

#include <algorithm>
#include <cmath>

constexpr float PI = acos(-1);

class Follower {
public:
    
    Follower(ros::NodeHandle n) : n(n) {
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);
        laser_sub = n.subscribe("scan", 1, &Follower::laserCallback, this);
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
        
        ROS_DEBUG_STREAM("min=" << *it << ",minindex=" << 
                minindex << ",rightangle=" << rightangle);
        geometry_msgs::TwistStamped move;

	move.header = msg->header;

        if (rightangle > PI/2 || rightangle < -PI/2) {
            move.twist.linear.x = 0;
        } else {
            move.twist.linear.x = 0.15;
        }

        move.twist.linear.y = 0;
        move.twist.linear.z = 0;

        move.twist.angular.x = 0;
        move.twist.angular.y = 0;
        move.twist.angular.z = rightangle;

        movement_pub.publish(move);
    }

private:
    ros::NodeHandle n;
    ros::Publisher movement_pub;
    ros::Subscriber laser_sub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "follower");
    ros::NodeHandle n;

    Follower planner(n);
    ROS_INFO("Follower setup successfully.");

    ros::spin();

    return 0;
}
