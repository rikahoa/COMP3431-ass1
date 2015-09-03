#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "odom_clean");
    ros::NodeHandle n;

    ros::Publisher clean_odom_pub = n.advertise<nav_msgs::Odometry>("/ass1/odom", 1);
    
    // put transforms here


    while (n.ok()) {
        tf::TransformListener listener;
        try {
            listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0)); 

            geometry_msgs::PoseStamped transformed_pose;

            listener.transformPose("/base_link", /* our point */, transformed_pose);

            //listener.lookupTransform("/base_link", "/map", ros::Time(0), transform);
        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
        }

        ros::spinOnce();
    }

    return 0;
}
