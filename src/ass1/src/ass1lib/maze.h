#ifndef _ASS1_MAZE_H
#define _ASS1_MAZE_H

#include <string>
#include <iostream>
#include <vector>
#include <utility>
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

    vector<pair<int, int>> path_to_grid(const vector<pair<int, int>>& astar_path) {
        vector<pair<int, int>> grid_path;

        for (const auto& a : astar_path) {
            int grid_x = static_cast<int>(a.first * og.info.resolution);
            int grid_y = static_cast<int>(a.second * og.info.resolution);

            if (!grid_path.empty() && grid_path.back().first == grid_x &&
                    grid_path.back().second == grid_y) {
                continue;
            }

            grid_path.push_back(pair<int, int>(grid_x, grid_y));
        }

        return grid_path;
    }
    
    int get_data(int x, int y) {
        return this->og.data[x * this->og.info.height + y];
    }
private:
    nav_msgs::OccupancyGrid og;    
};

#endif
