#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class Waypoint {
public:
    Waypoint(ros::NodeHandle n) : n(n) {
    }

private:
    ros::NodeHandle n;
    ros::Publisher movement_pub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n;

    Waypoint waypoint(n);

    ros::spin();

    return 0;
}
