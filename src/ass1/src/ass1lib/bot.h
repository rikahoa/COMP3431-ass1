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

    pair<double, double> get_position() {
        return make_pair(this->pose.position.x, this->pose.position.y);
    }

    double get_yaw() {
        return tf::getYaw(this->pose.orientation);
    }

    // Returns displacement in (distance, angle)
    pair<double, double> get_displacement(double x, double y) {
        // Get the vector to the robot.
        double vy = y - this->pose.position.y;
        double vx = x - this->pose.position.x;
        
        double distance = sqrt(vx*vx + vy*vy);

        // Find the angle.
        ROS_INFO_STREAM("atan: " << atan2(vx,vy) << "," "yaw: " << this->get_yaw());
        double target_angle = atan2(vx, vy) - this->get_yaw();

        return make_pair(distance, target_angle);
    }

    void update(const nav_msgs::Odometry::ConstPtr &odom) {
        this->pose = odom->pose.pose;
        this->_valid = true;
    }

    bool valid() {
        return this->_valid;
    }

    // Sets up a valid movement twist.
    void setup_movement(const pair<double,double>& target, geometry_msgs::Twist& move) {
        move.linear.x = move.linear.y = move.linear.z = 0;
        move.angular.x = move.angular.y = move.angular.z = 0;

        auto displacement = this->get_displacement(target.first, target.second);
        
        ROS_INFO_STREAM("target of " << target.first << "," << target.second);
        ROS_INFO_STREAM("we are at " << this->get_position().first << "," 
                << this->get_position().second);
        ROS_INFO_STREAM("angle change of " << displacement.second << " required.");
        ROS_INFO_STREAM("distance from target is " << displacement.first);

        if (displacement.second > 0.1 || displacement.second < -0.1) {
            // TODO: Make this better
            move.angular.z = 2*displacement.second; 
        } else {
            if (displacement.first > 0.1) {
                move.linear.x = 0.4;
            }
        } 
    }

    pair<int, int> get_og_coord(const Maze &m) {
        auto og = m.get_occupancy_grid();
        auto origin = og.info.origin.position;
        return make_pair(static_cast<int>((this->pose.position.x - origin.x) / og.info.resolution),
                         static_cast<int>((this->pose.position.y - origin.y) / og.info.resolution));
    }
    
private:
    geometry_msgs::Pose pose;
    bool _valid;
};

#endif 
