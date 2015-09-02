#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include <algorithm>
#include <cmath>

constexpr float PI = acos(-1);

class Planner {
public:
    
    Planner(ros::NodeHandle n) : n(n) {
    }

private:
    ros::NodeHandle n;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;

    Planner planner(n);
    ROS_INFO("Planner setup successfully.");

    ros::spin();

    return 0;
}
