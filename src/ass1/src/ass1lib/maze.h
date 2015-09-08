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
        vector<pair<double, double>> real_path;        
        
        for(auto it = astar_path.begin(); it!= astar_path.end(); ++it) {
            real_path.push_back(get_world_pos(*it));
        }
        
        vector<pair<double,double>> simplified = rdp_simplify(real_path, 0.1);
        
        std::queue<pair<double,double>> q(std::deque<pair<double,double>>(simplified.begin(),
                                                                  simplified.end()));
        return q;
      
    }
    
    static double distance_line_point(pair<pair<double,double>,pair<double,double>> line, pair<double,double> p0) {
        pair<double,double> p1 = line.first;
        pair<double,double> p2 = line.second;

        return  fabs((p2.second - p1.second)*p0.first - (p2.first - p1.first)*p0.second + p2.first*p1.second - p2.second*p1.first)/sqrt((p2.second-p1.second)*(p2.second-p1.second) - (p2.first-p1.first)*(p2.first-p1.first));
    }
    
    static vector<pair<double,double>> rdp_simplify(vector<pair<double, double>> in, double threshold) {
        vector<pair<double, double>> out;
        if (in.size() > 2) {
            //
            //  Find the vertex farthest from the line defined by the start and and of the path
            //

            double maxDist = 0;
            size_t maxDistIndex = 0;      
            pair<pair<double,double>,pair<double,double>> line = make_pair( *in.begin(), *(in.end()-1));

            for ( size_t i = 0;i < in.size(); i++ )
            {
                double dist = distance_line_point(line, in[i]);
                if ( dist > maxDist )
                {
                    maxDist = dist;
                    maxDistIndex = i;
                }
            }


            //
            //  If the farthest vertex is greater than our threshold, we need to
            //  partition and optimize left and right separately
            //

            if ( maxDist > threshold ) {
                //
                //  Partition 'in' into left and right subvectors, and optimize them
                //

                vector<pair<double, double>> left;
                vector<pair<double, double>> right;
                
                for ( size_t i = 0; i < maxDistIndex + 1; i++ ) left.push_back( in[i] );
                for ( size_t i = maxDistIndex; i < in.size(); i++ ) right.push_back( in[i] );


                vector<pair<double, double>> leftSimplified = rdp_simplify(left, threshold );
                vector<pair<double, double>> rightSimplified = rdp_simplify(right, threshold);

                //
                //  Stitch optimized left and right into 'out'
                //

                out.clear();
                for ( size_t i = 0; i < leftSimplified.size(); i++ ) out.push_back(leftSimplified[i]);
                for ( size_t i = 1; i < rightSimplified.size(); i++ ) out.push_back( rightSimplified[i] );
            } else  {
                out.push_back( line.first );
                out.push_back( line.second );
            }
            return out;
        } else {
            return in;
        }

    }    
    

    void rviz(
            ros::Publisher& pub, 
            const vector<pair<int,int>>& points, 
            const vector<pair<double,double>>& real_points) {
        nav_msgs::OccupancyGrid copy = this->og;
        for (const auto& p : points) {
            copy.data[p.second * copy.info.width + p.first] = 50; 
        }
        for (const auto& rp : real_points) {
            auto p = this->get_og_pos(rp);
            copy.data[p.second * copy.info.width + p.first] = 50; 
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
};

#endif
