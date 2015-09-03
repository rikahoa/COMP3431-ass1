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
        //We copy the grid and make a new version with fatter walls
        this->og = og;
        
        for(int x = 0;  x < this->og.info.width; ++x) {
            for(int y = 0;  y < this->og.info.height; ++y) {
                //Want the original values
                if( og.data[x * og.info.height + y] > 80) {
                    //find its neighbours within 5 squares, 
                    auto neighbours = get_fattened_neighbours(x,y, 5);
                    for(auto n: neighbours) {
                        set_data(n.first, n.second,100); 
                    } 
                }
            }
        }
    }
    
    vector<pair<int, int>> get_fattened_neighbours(int x, int y, int padding) {
        vector<pair<int, int>> neighbours;
        for(int i=0; i < padding; ++i) {
            if (x+i < this->og.info.width) {
                neighbours.push_back(pair<int, int>(x+i, y));
            }
            if (y+i < this->og.info.height) {
                neighbours.push_back(pair<int, int>(x, y+i));
            }
            if (y-i >=0) {
                neighbours.push_back(pair<int, int>(x, y-i));
            }
            if (x-i >=0) {
                neighbours.push_back(pair<int, int>(x-i, y));
            }
            for(int j = 0; j < padding; ++j) {
                if (x+i < this->og.info.width && y+j < this->og.info.height) {
                    neighbours.push_back(pair<int, int>(x+i, y+j));
                }
                if (x+i < this->og.info.width && y-j >=0) {
                    neighbours.push_back(pair<int, int>(x+i, y-j));
                }
                if (x-i >=0 && y+j < this->og.info.height) {
                    neighbours.push_back(pair<int, int>(x-i, y+j));
                }
                if (x-i >=0 && y-j >=0 ) {
                    neighbours.push_back(pair<int, int>(x-i, y-j));
                }
            }
        }
        
        
        return  vector<pair<int, int>>();
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
    
    void set_data(int x, int y, int value) {
        this->og.data[x * this->og.info.height + y] = value;
    }
    
    int get_data(int x, int y) {
        return this->og.data[x * this->og.info.height + y];
    }
private:
    nav_msgs::OccupancyGrid og;    
};

#endif
