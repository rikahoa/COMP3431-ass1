#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class Exploration {
    public:
        
        Exploration(ros::NodeHandle n) : n(n) {
            movement_pub = n.advertise<geometry_msgs::Twist>("/ass1/movement", 1);
        }

    private:
        ros::NodeHandle n;
        ros::Publisher movement_pub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    Exploration exploration(n);

    ros::spin();

    return 0;
}
