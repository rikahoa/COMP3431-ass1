#ifndef _ASS1_BOT_H
#define _ASS1_BOT_H

#include <utility>
#include "nav_msgs/Odometry.h"
#include "maze.h"
#include <sys/time.h>

using namespace std;

class Bot {
public:
    Bot() : x(0), y(0), angle(0) {
    }

    pair<double, double> get_position() {
        return make_pair(x, y);
    }

    void update(const nav_msgs::Odometry::ConstPtr &odom) {
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
