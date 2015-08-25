#include "ros/ros.h"

class Waypoint {
    public:
        
        Waypoint(ros::NodeHandle n) : n(n) {
        }

    private:
        ros::NodeHandle n;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n;

    ros::spin();

    return 0;
}
