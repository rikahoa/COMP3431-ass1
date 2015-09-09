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
    Maze() : _valid(false) {}

    Maze(int fatten_value) : fatten_value(fatten_value), _valid(false) {
        ROS_INFO("Fatten value is %d", fatten_value);
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
        vector<pair<double, double>> real_path;        
        
       ROS_INFO_STREAM("ORIGINAL PATH");
        for(auto it = astar_path.begin(); it!= astar_path.end(); ++it) {
            real_path.push_back(get_world_pos(*it));
            ROS_INFO_STREAM(get_world_pos(*it).first << "," << get_world_pos(*it).second);
        }
            
        vector<pair<double,double>> simplified = rdp_simplify(real_path, 0.05);
        
        ROS_INFO_STREAM("Simplified PATH");
        for(auto it = simplified.begin(); it!= simplified.end(); ++it) {
            ROS_INFO_STREAM((*it).first << "," << (*it).second);
        }

        std::queue<pair<double,double>> q(std::deque<pair<double,double>>(simplified.begin(),
                                                                  simplified.end()));
        return q;
      
    }
    
    static double distance_line_point(
            pair<pair<double,double>, pair<double,double>> line, 
            pair<double,double> p0) 
    {
        pair<double,double> p1 = line.first;
        double x1 = p1.first;
        double y1 = p1.second;
        pair<double,double> p2 = line.second;
        double x2 = p2.first;
        double y2 = p2.second;
        double x0 = p0.first;
        double y0 = p0.second;

        return fabs((y2-y1) * x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / 
            sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1));
    }
    
    static vector<pair<double,double>> rdp_simplify(
            vector<pair<double, double>> in, 
            double threshold) 
    {
        vector<pair<double, double>> out;
        if (in.size() > 2) {
            // Find the vertex farthest from the line defined by the start and and of the path
            double max_dist = 0;
            size_t max_dist_i = 0;      

            pair<pair<double,double>,pair<double,double>> line = make_pair(in.front(), in.back());
    
            for (size_t i = 0; i < in.size(); i++) {
                double dist = distance_line_point(line, in[i]);
                if (dist > max_dist) {
                    max_dist = dist;
                    max_dist_i = i;
                }
            }

            // If the farthest vertex is greater than our threshold, we need to
            // partition and optimize left and right separately
            if (max_dist > threshold) {
                // Partition 'in' into left and right subvectors, and optimize them
                vector<pair<double, double>> left;
                vector<pair<double, double>> right;
                
                for (size_t i = 0; i < max_dist_i + 1; i++) {
                    left.push_back(in[i]);
                }
                for (size_t i = max_dist_i; i < in.size(); i++) { 
                    right.push_back(in[i]);
                }

                vector<pair<double, double>> leftSimplified = rdp_simplify(left, threshold);
                vector<pair<double, double>> rightSimplified = rdp_simplify(right, threshold);

                // Stitch optimized left and right into 'out'
                out.clear();
                for (size_t i = 0; i < leftSimplified.size(); i++) {
                    out.push_back(leftSimplified[i]);
                }
                for (size_t i = 1; i < rightSimplified.size(); i++) {
                    out.push_back(rightSimplified[i]);
                }
            } else  {
                out.push_back(line.first);
                out.push_back(line.second);
            }
            return out;
        } else {
            return in;
        }
    }    
    
    void rviz(
            ros::Publisher& pub, 
            const vector<pair<int,int>>& points, 
            const vector<pair<double,double>>& real_points) 
    {
        nav_msgs::OccupancyGrid copy = this->og;
        for (const auto& p : points) {
            copy.data[p.second * copy.info.width + p.first] = 40; 
        }
        for (const auto& rp : real_points) {
            auto p = this->get_og_pos(rp);
            copy.data[p.second * copy.info.width + p.first] = 75; 
        }
        pub.publish(copy);
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
    int fatten_value;
};

#endif
