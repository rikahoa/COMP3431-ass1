#ifndef _ASS1_MAZE_H
#define _ASS1_MAZE_H

#include <string>
#include <iostream>
#include <vector>
#include <utility>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <queue>

using namespace std;

class Maze {
public:
    Maze() : _valid(false) {
    
    }

    const nav_msgs::OccupancyGrid& get_occupancy_grid() const {
        return this->og;
    }

    void set_occupancy_grid(const nav_msgs::OccupancyGrid &og) {
        this->og = og;
        fatten_neighbours(og);

        this->_valid = true;
    }

    bool valid() {
        return this->_valid;
    }

    queue<pair<double, double>> og_to_real_path(const vector<pair<int, int>>& astar_path) const {
        queue<pair<double, double>> real_path;
        
        if (astar_path.size() > 0) {
            // push out the start one.
            pair<int, int> start = astar_path.front();
            real_path.push(this->get_world_pos(start));

            // push in similar elements.
            for (auto it = ++astar_path.begin(); it != astar_path.end(); ++it) {
                // as long as one of them matches the previous, we take this path.
                if (start.first != it->first && start.second != it->second) {
                    real_path.push(this->get_world_pos(*it));
                    start = *it;
                }
            }
           
            // flush out the back
            if (start != astar_path.back()) {
                real_path.push(this->get_world_pos(astar_path.back()));
            }
        }

        return real_path;
    }

    // RIP Aneita
    vector<pair<int, int>> path_to_grid(const vector<pair<int, int>>& astar_path) const {
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
    
    void set_data(int x, int y, int value) {
        this->og.data[y * this->og.info.width + x] = value;
    }
    
    int get_data(int x, int y) const {
        return this->og.data[y * this->og.info.width + x];
    }
    
    double get_resolution() const {
        return this->og.info.resolution;
    }

    int get_height() const {
        return this->og.info.height;
    }

    int get_width() const {
        return this->og.info.width;
    }

    pair<double, double> get_world_pos(pair<int,int> pos) const {
       return make_pair(pos.first*og.info.resolution + og.info.origin.position.x, 
               pos.second*og.info.resolution + og.info.origin.position.y);
    }
    
    pair<int, int> get_og_pos(pair<double,double> pos) const {
        auto origin = this->og.info.origin.position;
        return make_pair(static_cast<int>((pos.first - origin.x) / this->og.info.resolution),
                         static_cast<int>((pos.second - origin.y) / this->og.info.resolution));
    }
private:
    void fatten_neighbours(const nav_msgs::OccupancyGrid &og);
    
    nav_msgs::OccupancyGrid og;   
    static const vector<pair<int, int>> DIRECTIONS;
    bool _valid;
};

#endif
