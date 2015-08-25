#include "ros/ros.h"

class Beacon {
    public:
        
        Beacon(ros::NodeHandle n) : n(n) {
        }

    private:
        ros::NodeHandle n;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "beacon");
    ros::NodeHandle n;

    ros::spin();

    return 0;
}
