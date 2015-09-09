#include "maze.h"

const vector<pair<int,int>> Maze::DIRECTIONS = 
    vector<pair<int, int>>{make_pair(-1,0),make_pair(1,0),make_pair(0,-1),make_pair(0,1),
                           make_pair(-1,-1),make_pair(1,1),make_pair(1,-1),make_pair(-1,1)};

#define MIN_PROB 95
void Maze::fatten_neighbours(const nav_msgs::OccupancyGrid &og) {
    vector<vector<bool>> seen;
    for (size_t y = 0; y < og.info.height; ++y) {
        seen.push_back(vector<bool>(og.info.width));
    }

    queue<pair<pair<size_t, size_t>, size_t>> bfs;
    for (size_t y = 0; y < og.info.height; ++y) {
        for (size_t x = 0; x < og.info.width; ++x) {
            if (og.data[y * og.info.width + x] >= MIN_PROB) {
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
       
        if (seen[y][x] || distance > fatten_value) {
            continue;
        }

        // Mark data
        seen[y][x] = true;
        set_data(x, y, 100);

        // Search all directions if not too fat.
        if (distance < fatten_value) {
            for (const auto& dir : Maze::DIRECTIONS) {
                auto newX = x + dir.first;
                auto newY = y + dir.second;
                if (newX >= 0 && newX < og.info.width && newY >= 0 && newY < og.info.height &&
                        !seen[newY][newX]) {
                    bfs.push(make_pair(make_pair(newX, newY), distance + 1));
                }
            }
        }
    }
}
