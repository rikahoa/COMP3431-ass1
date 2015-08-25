#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"

#include <algorithm>
#include <cmath>

constexpr float PI = acos(-1);

class Planner {
    public:
        
        Planner(ros::NodeHandle n) : n(n) {
            laser_sub = n.subscribe("scan", 1, &Planner::laser_callback, this);
            map_sub = n.subscribe("map", 1, &Planner::map_callback, this);
            
            movement_pub = n.advertise<geometry_msgs::Twist>("/ass1/movement", 1);
        }

        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
            ROS_INFO_STREAM("I HAVE A MAP");
        }

        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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
            geometry_msgs::Twist move;

            if (rightangle > PI/2 || rightangle < -PI/2) {
                move.linear.x = 0;
            } else {
                move.linear.x = 0.15;
            }

            move.linear.y = 0;
            move.linear.z = 0;

            move.angular.x = 0;
            move.angular.y = 0;
            move.angular.z = rightangle;

            movement_pub.publish(move);
        }

    private:
        ros::NodeHandle n;
        ros::Publisher movement_pub;
        ros::Subscriber laser_sub;
        ros::Subscriber map_sub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    Planner planner(n);
    ROS_INFO("Planner setup successfully.");

    ros::spin();

    return 0;
}
