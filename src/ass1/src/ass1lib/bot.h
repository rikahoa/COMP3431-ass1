#ifndef _ASS1_BOT_H
#define _ASS1_BOT_H

#include <utility>
#include "nav_msgs/Odometry.h"
#include "maze.h"
#include "tf/transform_listener.h"
#include "tf/tf.h"
#include "ros/ros.h"
#include <sys/time.h>

using namespace std;

    tf::TransformListener listener;
class Bot {
public:
    Bot() : x(0), y(0), angle(0) {
    }

    pair<double, double> get_position() {
        return make_pair(x, y);
    }

    void update(const nav_msgs::Odometry::ConstPtr &odom) {
        tf::StampedTransform transform;
        listener.lookupTransform("/map", "/base_link", odom->header.stamp, transform);
        this->x = transform.getOrigin().x();
        this->y = transform.getOrigin().y();
        // TODO: angle

        ROS_INFO_STREAM("POINT: " << x << "," << y);
    }

    pair<int, int> get_occupancy_grid_coord(double resolution) {
        return make_pair(static_cast<int>((this->x) / resolution),
                         static_cast<int>((this->y) / resolution));
    }
private:
    double x;
    double y;
    double angle;
};

#endif 
