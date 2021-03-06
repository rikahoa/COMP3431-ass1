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

    pair<double, double> get_displacement(const pair<double,double>& target) const {
        // Get the vector to the robot.
        double vy = target.second - this->pose.position.y;
        double vx = target.first - this->pose.position.x;
        
        double distance = sqrt(vx*vx + vy*vy);

        // Find the angle to target.
        double target_angle = atan2(vy, vx) - this->get_yaw();
        // fix the angle
        target_angle -= static_cast<int>(target_angle / (2*PI)) * 2 * PI;
        // fix to spin the right way around
        if (target_angle > PI) {
            target_angle = -target_angle + PI;
        } else if (target_angle < -PI) {
            target_angle = -target_angle - PI;
        }

        return make_pair(distance, target_angle);
    }

    // Sets up a valid movement twist.
    void setup_spin(geometry_msgs::Twist& move, double delta) {
        move.linear.x = move.linear.y = move.linear.z = 0;
        move.angular.x = move.angular.y = 0;
        move.angular.z = std::min(0.6, std::max(-0.6, 2*delta));
    }

    void setup_movement(const pair<double,double>& target, geometry_msgs::Twist& move) {
        move.linear.x = move.linear.y = move.linear.z = 0;
        move.angular.x = move.angular.y = move.angular.z = 0;
        
        ROS_DEBUG_STREAM("** we are at " << this->get_position().first << "," 
                << this->get_position().second);
        ROS_DEBUG_STREAM("** target of " << target.first << "," << target.second);

        auto displacement = get_displacement(target);
        double distance = displacement.first;
        double target_angle = displacement.second;

        ROS_DEBUG_STREAM("** angle: " << (target_angle + this->get_yaw()) 
                << " yaw: " << this->get_yaw());
        ROS_DEBUG_STREAM("** angle change of " << target_angle << " required.");
        ROS_DEBUG_STREAM("** distance from target is " << distance);

        if (fabs(target_angle) > 0.2) {
            move.angular.z = std::max(-0.6, std::min(0.6, 3*target_angle)); 
        } else {
            if (distance > 0.1) {
                move.linear.x = 0.25;
            }
        }

        ROS_DEBUG_STREAM("** movement set to " << move.linear.x << "," << move.angular.z);
    }

    double distance(const pair<double, double>& target) const {
        auto vx = target.first - this->pose.position.x;
        auto vy = target.second - this->pose.position.y;
        return sqrt(vx*vx + vy*vy);
    }

    pair<int, int> get_og_pos(const Maze &m) const {
        return m.get_og_pos(make_pair(this->pose.position.x, this->pose.position.y));
    }
   
    bool close_enough(const pair<double, double>& target) const {
        auto displacement = get_displacement(target);
        return displacement.first < 0.1;
    }
    
    bool beacon_close_enough(const pair<double, double>& target) const {
        auto displacement = get_displacement(target);
        return displacement.first < 0.25;
    }

    bool astar_okay(const pair<double, double>& target) const {
        auto displacement = get_displacement(target);
        return displacement.first > 0.3;
    }
private:
    geometry_msgs::Pose pose;
    bool _valid;
};

#endif 
