#include "ros/ros.h"

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

    ros::spin();

    return 0;
}
