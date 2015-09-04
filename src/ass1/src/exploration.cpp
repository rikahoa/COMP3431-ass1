#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "ass1lib/astar.h"
#include "ass1lib/maze.h"
#include "ass1lib/bot.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include "geometry_msgs/TwistStamped.h"
#include <random>

#define EXPLORE_THRESHOLD 1.0

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, nav_msgs::Odometry> ApproxPolicy;

class ExplorationState : public State {
public:
    ExplorationState(int x, int y, int cost) : 
        ExplorationState(x, y, cost, make_pair(-1, -1)) {};

    virtual bool is_goal(const Maze& maze) const override {
        return maze.get_occupancy_grid().data[y * maze.get_occupancy_grid().info.height + x] == -1 &&
            this->cost > EXPLORE_THRESHOLD;
    }

    virtual vector<State*> explore(const Maze& maze, 
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        
        for (const auto &p : State::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;
            if (x >= 0 && x < maze.get_width() && 
                    y >= 0 && y < maze.get_height() && 
                    check(make_pair(x, y)) &&
                    maze.get_data(x, y) <= 0) {
                new_states.push_back(
                        new ExplorationState(x, y, 
                            this->cost + maze.get_resolution(), this->get_position()));
            }
        }
        return new_states;
    }
private:
    ExplorationState(int x, int y, int cost, pair<int, int> parent) :
        State(x, y, cost, parent, 0) {};
};


class Exploration {
public:
    Exploration(ros::NodeHandle n) : n(n),
        map_sub(n, "/map", 1), 
        odom_sub(n, "/ass1/odom", 1), 
        sync(ApproxPolicy(10), map_sub, odom_sub)  
    {
        
        sync.registerCallback(boost::bind(&Exploration::map_callback, this, _1, _2)); 
        
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);

    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og, 
            const nav_msgs::Odometry::ConstPtr &odom) {
        this->maze.set_occupancy_grid(*og);
        this->bot.update(odom);

        // Find current position.
        auto og_pos = this->bot.get_og_coord(this->maze);
        ROS_INFO_STREAM("og point: " << og_pos.first << "," << og_pos.second);
        
        // Do a A* to the nearest frontier
        auto path = search(this->maze, new ExplorationState(og_pos.first, og_pos.second, 0));
        
        auto targetingrid = *(path.begin() + 3);
        auto target = this->maze.get_world_coord(targetingrid);

        // Generate me a message.
        geometry_msgs::TwistStamped move;
        move.header = odom->header;
        move.twist.linear.x = move.twist.linear.y = move.twist.linear.z = 0;
        move.twist.angular.x = move.twist.angular.y = move.twist.angular.z = 0;

        auto displacement = this->bot.get_displacement(target.first, target.second);
        
        ROS_INFO_STREAM("target of " << target.first << "," << target.second);
        ROS_INFO_STREAM("we are at " << this->bot.get_position().first << "," 
                << this->bot.get_position().second);
        ROS_INFO_STREAM("angle change of " << displacement.second << " required.");
        ROS_INFO_STREAM("distance from target is " << displacement.first);

        if (displacement.second > 0.1 || displacement.second < -0.1) {
            // TODO: Make this better
            move.twist.angular.z = 2*displacement.second; 
        } else {
            if (displacement.first > 0.1) {
                move.twist.linear.x = 0.4;
            }
        } 

        movement_pub.publish(move);

    }
private:
    Maze maze;
    Bot bot;

    ros::NodeHandle n;
    ros::Publisher movement_pub;
    
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<ApproxPolicy> sync;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    Exploration exploration(n);
    ros::spin();

    return 0;
}
