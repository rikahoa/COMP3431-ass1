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
    tf::TransformListener listener;
    tf::StampedTransform transform;
    
    while (n.ok()) {
        try {
            nav_msgs::Odometry odom;
            odom.header.frame_id = transform.frame_id_;
            odom.child_frame_id = transform.child_frame_id_;

            ros::Time now = ros::Time::now();
            if (listener.waitForTransform("/map", "/base_link", now, ros::Duration(1.0))) {
                listener.lookupTransform("/map", "/base_link", now, transform);
                odom.header.stamp = transform.stamp_;
            } else {
                odom.header.stamp = ros::Time::now();
            }

            odom.pose.pose.position.x = transform.getOrigin().x();
            odom.pose.pose.position.y = transform.getOrigin().y();
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation.x = transform.getRotation().x();
            odom.pose.pose.orientation.y = transform.getRotation().y();
            odom.pose.pose.orientation.z = transform.getRotation().z();
            odom.pose.pose.orientation.w = transform.getRotation().w();
            
            clean_odom_pub.publish(odom);
        } catch (tf::TransformException &ex) {
            ROS_ERROR("Error! %s", ex.what());
        } 
            
        ros::spinOnce();
    }

    return 0;
}
