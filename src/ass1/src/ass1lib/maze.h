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
    Maze() {
    
    }

    const nav_msgs::OccupancyGrid& get_occupancy_grid() const {
        return this->og;
    }

    void set_occupancy_grid(const nav_msgs::OccupancyGrid &og) {
        this->og = og;

        fatten_neighbours(og);
    }

    #define FATTEN 4
    #define MIN_PROB 80
    void fatten_neighbours(const nav_msgs::OccupancyGrid &og) {
        vector<vector<bool>> seen;
        for (int y = 0; y < og.info.height; ++y) {
            seen.push_back(vector<bool>(og.info.width));
        }

        queue<pair<pair<int, int>, int>> bfs;
        for (int y = 0; y < this->og.info.height; ++y) {
            for (int x = 0; x < this->og.info.width; ++x) {
                if (og.data[y * og.info.height + x] >= 50) {
                    bfs.push(make_pair(make_pair(x, y), 0));
                }
            }
        }

        // run through all possible positions.
        while (!bfs.empty()) {
            auto info = bfs.front();
            bfs.pop();
            
            // Find information.
            auto x = info.first.first;
            auto y = info.first.second;
            auto distance = info.second;
           
            if (seen[y][x] || distance > FATTEN) {
                continue;
            }

            // Mark data
            seen[y][x] = true;
            set_data(y, x, 100);

            // search all directions
            for (const auto& dir : Maze::DIRECTIONS) {
                auto newX = x + dir.first;
                auto newY = y + dir.second;
                if (newX >= 0 && newX < og.info.width && newY >= 0 && newY < og.info.height &&
                        !seen[newY][newX] && distance < FATTEN) {
                    bfs.push(make_pair(make_pair(newX, newY), distance + 1));
                }
            }
        }
    }

    queue<pair<double, double>> og_to_real_path(const vector<pair<int, int>>& astar_path) {
        queue<pair<double, double>> real_path;
        for (const auto &point : astar_path) {
            real_path.push(this->get_world_pos(point));
        }
        return real_path;
    }

    // RIP Aneita
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
        this->og.data[y * this->og.info.height + x] = value;
    }
    
    int get_data(int x, int y) const {
        return this->og.data[y * this->og.info.height + x];
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

    pair<double, double> get_world_pos(pair<int,int> coord) const {
       return make_pair(coord.first*og.info.resolution + og.info.origin.position.x, 
               coord.second*og.info.resolution + og.info.origin.position.y);
    }
private:
    nav_msgs::OccupancyGrid og;   
    static const vector<pair<int, int>> DIRECTIONS;
};

#endif
