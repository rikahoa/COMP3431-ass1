#ifndef _ASS1_BOT_H
#define _ASS1_BOT_H

#include <utility>
#include "nav_msgs/Odometry.h"
#include "maze.h"

using namespace std;

class Bot {
public:
    Bot() : x(0), y(0), angle(0) {}

    pair<double, double> get_position() {
        return make_pair(x, y);
    }

    void update(const nav_msgs::Odometry::ConstPtr &odom) {
        this->x = odom->pose.pose.position.x;
        this->y = odom->pose.pose.position.y;
        this->angle = odom->twist.twist.angular.z;
    }

    pair<int, int> get_occupancy_grid_coord(const Maze &maze) {
        auto og = maze.get_occupancy_grid();
        auto origin = og.info.origin.position;
        auto resolution = og.info.resolution;
        ROS_INFO_STREAM("origin: " << origin.x << "," << origin.y << ",res=" << resolution);
        ROS_INFO_STREAM("where we are: " << this->x << "," << this->y);
        return make_pair(static_cast<int>((this->x - origin.x) / resolution),
                         static_cast<int>((this->y - origin.y) / resolution));
    }
private:
    double x;
    double y;
    double angle;
};

#endif 
