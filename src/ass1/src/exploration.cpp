#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "ass1lib/astar.h"
#include "ass1lib/maze.h"
#include "ass1lib/bot.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

// Must travel at least this far.
#define EXPLORE_THRESHOLD 0.5
#define CLOSE_ENOUGH 0.1

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::OccupancyGrid, nav_msgs::Odometry> ApproxPolicy;

class ExplorationState : public State {
public:
    ExplorationState(int x, int y, double cost) : 
        ExplorationState(x, y, cost, make_pair(-1, -1)) {};

    virtual bool is_goal(const Maze& maze) const override {
        return maze.get_data(this->x, this->y) == -1 &&
            this->cost >= EXPLORE_THRESHOLD;
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
    ExplorationState(int x, int y, double  cost, pair<int, int> parent) :
        State(x, y, cost, parent, 0) {};
};


class Exploration {
public:
    Exploration(ros::NodeHandle n) : 
        started(false),
        n(n),
        map_sub(n, "/map", 1), 
        odom_sub(n, "/ass1/odom", 1), 
        sync(ApproxPolicy(10), map_sub, odom_sub)  
    {
        sync.registerCallback(boost::bind(&Exploration::map_callback, this, _1, _2)); 
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);

    }

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og, 
            const nav_msgs::Odometry::ConstPtr &odom) {
        this->maze.set_occupancy_grid(*og);
        this->bot.update(odom);

        // Continue while the goal is unknown.
        if (!started || this->maze.get_data(og_target.first, og_target.second) == -1) {
            // Find current position.
            auto og_pos = this->bot.get_og_coord(this->maze);
            ROS_INFO_STREAM("position: " << bot.get_position().first << "," 
                    << bot.get_position().second);
            ROS_INFO_STREAM("og point: " << og_pos.first << "," << og_pos.second);
        
            ROS_INFO_STREAM("Target probability found. Commencing astar.");
            // Do a A* to the nearest frontier
            auto og_path = search(this->maze, new ExplorationState(og_pos.first, og_pos.second, 0));
            if (og_path.empty()) {
                ROS_ERROR_STREAM("Empty path to target.");
                return;
            }
            ROS_INFO_STREAM("Converting into path data.");
            // Target the frontier in real coordinates.
            this->og_target = og_path.back();
            this->path = this->maze.og_to_real_path(og_path);
        }

        // Populate until next path is found.
        while (!this->path.empty() && this->bot.distance(path.front()) < CLOSE_ENOUGH) {
            path.pop();
        }

        // Error checking
        if (path.empty()) {
            ROS_ERROR_STREAM("Path empty! Cannot move anywhere...");
            return;
        }

        this->started = true;
        
        ROS_INFO_STREAM("We need to know " << og_target.first << "," << og_target.second << 
                "( world coord" << path.back().first << "," << path.back().second << ")");
        
        // Generate me a move message to target.
        geometry_msgs::TwistStamped move;
        move.header = odom->header;
        this->bot.setup_movement(path.front(), move.twist);
        movement_pub.publish(move);
    }

    bool started;
    Maze maze;
    Bot bot;

    ros::NodeHandle n;
    ros::Publisher movement_pub;
    
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<ApproxPolicy> sync;

    pair<int, int> og_target;
    queue<pair<double, double>> path;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    Exploration exploration(n);
    ros::spin();

    return 0;
}
