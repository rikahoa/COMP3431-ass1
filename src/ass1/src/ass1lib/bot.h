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
        
        ROS_INFO_STREAM("target of " << target.first << "," << target.second);

        // Get the vector to the robot.
        double vy = target.first - this->pose.position.y;
        double vx = target.second - this->pose.position.x;
        
        double distance = sqrt(vx*vx + vy*vy);

        // Find the angle to target.
        double target_angle = atan2(vx, vy) - this->get_yaw();

        ROS_INFO_STREAM("atan: " << atan2(vx,vy) << "," "yaw: " << this->get_yaw());
        ROS_INFO_STREAM("we are at " << this->get_position().first << "," 
                << this->get_position().second);
        ROS_INFO_STREAM("angle change of " << distance << " required.");
        ROS_INFO_STREAM("distance from target is " << distance);

        if (fabs(target_angle) > 0.1) {
            // TODO: Make this better
            move.angular.z = 2 * target_angle; 
        } else {
            if (distance > 0.1) {
                move.linear.x = 0.4;
            }
        } 
    }

    double distance(const pair<double, double>& target) {
        auto vx = target.first - this->pose.position.x;
        auto vy = target.second - this->pose.position.y;
        return sqrt(vx*vx + vy*vy);
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
