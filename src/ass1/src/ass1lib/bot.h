#ifndef _ASS1_BOT_H
#define _ASS1_BOT_H

#include <utility>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "maze.h"
#include <sys/time.h>
#include <tf/transform_datatypes.h>
#include <cmath>

using namespace std;

constexpr float PI = acos(-1);

class Bot {
public:
    Bot() : _valid(false) {
    }

    pair<double, double> get_position() const {
        return make_pair(this->pose.position.x, this->pose.position.y);
    }

    double get_yaw() const {
        tf::Quaternion q(this->pose.orientation.x, this->pose.orientation.y,
                this->pose.orientation.z, this->pose.orientation.w);
        q.normalize();
        return tf::getYaw(q);
    }

    void update(const nav_msgs::Odometry::ConstPtr &odom) {
        this->pose = odom->pose.pose;
        this->_valid = true;
    }

    bool valid() const {
        return this->_valid;
    }

    // Sets up a valid movement twist.
    void setup_movement(const pair<double,double>& target, geometry_msgs::Twist& move) {
        move.linear.x = move.linear.y = move.linear.z = 0;
        move.angular.x = move.angular.y = move.angular.z = 0;
        
        ROS_INFO_STREAM("** we are at " << this->get_position().first << "," 
                << this->get_position().second);
        ROS_INFO_STREAM("** target of " << target.first << "," << target.second);

        // Get the vector to the robot.
        double vy = target.first - this->pose.position.y;
        double vx = target.second - this->pose.position.x;
        
        double distance = sqrt(vx*vx + vy*vy);

        // Find the angle to target.
        double target_angle = atan2(vx, vy) - this->get_yaw();
        target_angle -= static_cast<int>(target_angle / PI) * PI;

        ROS_INFO_STREAM("** atan: " << atan2(vx,vy) << "," "yaw: " << this->get_yaw());
        ROS_INFO_STREAM("** angle change of " << target_angle << " required.");
        ROS_INFO_STREAM("** distance from target is " << distance);

        if (target_angle > 0.1) {
            // TODO: Make this better
            move.angular.z = 0.4; 
        } else if (target_angle < -0.1) {
            move.angular.z = -0.4;
        } else {
            if (distance > 0.1) {
                move.linear.x = 0.1;
            }
        }
        ROS_INFO_STREAM("** movement set to " << move.linear.x << "," << move.angular.z);
    }

    double distance(const pair<double, double>& target) const {
        auto vx = target.first - this->pose.position.x;
        auto vy = target.second - this->pose.position.y;
        return sqrt(vx*vx + vy*vy);
    }

    pair<int, int> get_og_pos(const Maze &m) const {
        return m.get_og_pos(make_pair(this->pose.position.x, this->pose.position.y));
    }
   
    bool close_enough(const pair<double, double>& target) {
        double vy = target.first - this->pose.position.y;
        double vx = target.second - this->pose.position.x;
        
        double distance = sqrt(vx*vx + vy*vy);
        double target_angle = atan2(vx, vy) - this->get_yaw();
        target_angle -= static_cast<int>(target_angle / PI) * PI;
        return distance < 0.1 && fabs(target_angle) < 0.1;
    }
private:
    geometry_msgs::Pose pose;
    bool _valid;
};

#endif 
