#include "ros/ros.h"

class Exploration {
    public:
        
        Exploration(ros::NodeHandle n) : n(n) {
        }

    private:
        ros::NodeHandle n;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    ros::spin();

    return 0;
}
