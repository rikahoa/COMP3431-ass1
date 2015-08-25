#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

class Movement {
    public:
        Movement(ros::NodeHandle n) : n(n) {
            navi_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
            movement_sub = n.subscribe("ass1/movement", 1, &Movement::movement_sub_callback, this);
        }

        void movement_sub_callback(const geometry_msgs::Twist::ConstPtr& msg) {
            ROS_DEBUG_STREAM("Moving x = " << msg->linear.x << ", angle z = " << msg->angular.z);
            navi_pub.publish(*msg);
        }
    private:
        ros::NodeHandle n;
        ros::Publisher navi_pub;
        ros::Subscriber movement_sub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle n;
    
    Movement movement(n);
    ROS_INFO("Movement setup successfully.");

    ros::spin();

    return 0;
}
