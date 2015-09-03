#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "odom_clean");
    ros::NodeHandle n;

    ros::Publisher clean_odom_pub = n.advertise<nav_msgs::Odometry>("/ass1/odom", 1);
    
    // put transforms here

    ros::Rate rate(1.0);

    tf::TransformListener listener;
    while (n.ok()) {
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

            nav_msgs::Odometry odom;
            odom.header.stamp = transform.stamp_;
            odom.header.frame_id = transform.frame_id_;
            odom.child_frame_id = transform.child_frame_id_;

            odom.pose.pose.position.x = transform.getOrigin().x();
            odom.pose.pose.position.y = transform.getOrigin().y();
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation.x = transform.getRotation().x();
            odom.pose.pose.orientation.y = transform.getRotation().y();
            odom.pose.pose.orientation.z = transform.getRotation().z();
            odom.pose.pose.orientation.w = transform.getRotation().w();

            clean_odom_pub.publish(odom);
        } catch (tf::TransformException ex) {
            ROS_ERROR("Error! %s", ex.what());
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
