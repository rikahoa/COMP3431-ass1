#ifndef _ASS1_MAZE_H
#define _ASS1_MAZE_H

#include <string>
#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

class Maze {
public:
    Maze() {
    
    }

    const nav_msgs::OccupancyGrid& get_occupancy_grid() const {
        return this->og;
    }

    void set_occupancy_grid(const nav_msgs::OccupancyGrid &og) {
        // make a copy of the grid
        this->og = og;
    }
private:
    nav_msgs::OccupancyGrid og;    
};

#endif
