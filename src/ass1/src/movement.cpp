#include "ros/ros.h"

class Movement {
    public:
        
        Movement(ros::NodeHandle n) : n(n) {
        }

    private:
        ros::NodeHandle n;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle n;

    ros::spin();

    return 0;
}
