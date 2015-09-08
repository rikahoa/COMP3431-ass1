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
#include "ass1/FoundBeacons.h"
#include <std_msgs/String.h>


using namespace std;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::OccupancyGrid, nav_msgs::Odometry> ApproxPolicy;

class ExplorationState : public State {
public:
    ExplorationState(int x, int y, double cost, Bot* bot) : 
        ExplorationState(x, y, cost, make_pair(-1, -1), bot) {};

    virtual bool is_goal(const Maze& maze) const override {
        return maze.get_data(this->x, this->y) == -1 &&
            this->bot->astar_okay(maze.get_world_pos(make_pair(this->x, this->y)));
    }

    virtual vector<State*> explore(const Maze& maze, 
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        for (const auto &p : State::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;

            // try move places.
            if (maze.get_data(this->x, this->y) < 80/* || 
                    this->bot->close_enough(maze.get_world_pos(make_pair(this->x, this->y)))*/) {
                if (x >= 0 && x < maze.get_width() && 
                        y >= 0 && y < maze.get_height() && 
                        check(make_pair(x, y))) {
                    new_states.push_back(
                            new ExplorationState(x, y, 
                                this->cost + maze.get_resolution(), this->get_position(), this->bot));
                }
            }
        }
        return new_states;
    }
private:
    ExplorationState(int x, int y, double cost, pair<int, int> parent, Bot* bot) :
        State(x, y, cost, parent, 0), bot(bot) {};
    Bot* bot;
};


class Exploration {
public:
    Exploration(ros::NodeHandle n) : 
        started(false),
        n(n)
    {
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);
        beacons_sub = n.subscribe("ass1/beacons", 1, &Exploration::beacon_callback, this);
        odom_sub = n.subscribe("ass1/odom", 1, &Exploration::odom_callback, this);
        recalc_sub = n.subscribe("ass1/recalc", 1, &Exploration::recalc_callback, this);
        map_sub = n.subscribe("map", 1, &Exploration::map_callback, this);
        map_fatten_pub = n.advertise<nav_msgs::OccupancyGrid>("/ass1/map", 1);
    }

private:
    void beacon_callback(const ass1::FoundBeacons::ConstPtr& msg) {
        ROS_INFO_STREAM("Beacons message on Exploration! Shutting down.");
        ros::shutdown();
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og) {
        this->maze.set_occupancy_grid(*og);
        // publish for debugging...
        this->maze.rviz(map_fatten_pub, this->og_path, vector<pair<double,double>>());
    }

    void recalculate_astar() {
        // Find current position.
        auto og_pos = this->bot.get_og_pos(this->maze);
    
        ROS_INFO_STREAM("* ASTAR invoked.");
        // Do a A* to the nearest frontier
        auto og_path = search(this->maze, 
                new ExplorationState(og_pos.first, og_pos.second, 0, &this->bot));
        
        // can't find path!
        if (og_path.empty()) {
            ROS_ERROR_STREAM("* No target found...");
            return;
        }

        // Target the frontier in real position.
        this->og_target = og_path.back();
        this->og_path = std::move(og_path);
        this->path = this->maze.og_to_real_path(this->og_path);
        this->started = true;
        this->maze.rviz(map_fatten_pub, this->og_path, vector<pair<double,double>>());
        ROS_DEBUG_STREAM("* Let's find " << og_target.first << "," << og_target.second);
    }

    void recalc_callback(const std_msgs::String::ConstPtr &msg) {
        ROS_INFO_STREAM("Recalculate whores!");
        recalculate_astar();
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
        this->bot.update(odom);

        if (this->maze.valid()) {
            ROS_INFO_STREAM("~~~~~~~~~~~ move fat bastard! ~~~~~~~~~~");
        
            // Continue while the goal is unknown.
            ROS_INFO_STREAM("maze getting " << og_target.first << "," << og_target.second 
                    << ":" << this->maze.get_data(og_target.first, og_target.second));
            if (!started || this->maze.get_data(og_target.first, og_target.second) > -1) {
                recalculate_astar();
            }

            // == REMOVE WHEN DONE
            queue<pair<double,double>> p(this->path);
            while (!p.empty()) {
                auto s = p.front();
                p.pop();
                ROS_INFO_STREAM("* path: " << s.first << "," << s.second);
            }
            // ==

            // Populate until next path is found.
            while (!this->path.empty() && this->bot.close_enough(path.front())) {
                ROS_INFO_STREAM("close enough to " << path.front().first << "," << 
                        path.front().second << " ... popping");
                path.pop();
            }

            // Error checking
            if (path.empty()) {
                ROS_WARN_STREAM("Exploration path empty! Cannot move anywhere...");
                recalculate_astar();
                return;
            }

            ROS_INFO_STREAM("We need to know " << og_target.first << "," << og_target.second << 
                    " (world pos " << path.back().first << "," << path.back().second << ")");
            
            // Generate me a move message to target.
            geometry_msgs::TwistStamped move;
            move.header = odom->header;
            this->bot.setup_movement(path.front(), move.twist);
            movement_pub.publish(move);
        }
    }

    bool started;
    Maze maze;
    Bot bot;

    ros::NodeHandle n;
    ros::Publisher movement_pub;    
    ros::Publisher map_fatten_pub;
    ros::Subscriber map_sub;
    ros::Subscriber recalc_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber beacons_sub;

    pair<int, int> og_target;
    queue<pair<double, double>> path;
    vector<pair<int, int>> og_path;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    Exploration exploration(n);
    ros::spin();

    return 0;
}
