#include "ros/ros.h"
#include <utility>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ass1lib/astar.h"
#include "ass1lib/maze.h"
#include "ass1lib/bot.h"

// Must travel at least this far.
#define EXPLORE_THRESHOLD 0.5
#define CLOSE_ENOUGH 0.1

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::OccupancyGrid, nav_msgs::Odometry> ApproxPolicy;

class WaypointState : public State {
public:
    WaypointState(int x, int y, double cost, double heuristic, pair<int,int> goal) : 
        WaypointState(x, y, cost, make_pair(-1, -1), heuristic, goal) {
        
    }

    virtual bool is_goal(const Maze& maze) const override {
        return x == goal.first && y == goal.second;
    }

    virtual vector<State*> explore(const Maze& maze,
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        auto world_goal = maze.get_world_pos(this->goal);
        for (const auto &p : State::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;

            if (x >= 0 && x < maze.get_width() && 
                    y >= 0 && y < maze.get_height() && 
                    check(make_pair(x, y)) &&
                    maze.get_data(x, y) <= 0) {
                auto pos = maze.get_world_pos(make_pair(x, y)); 
                double vx = pos.first - world_goal.first;
                double vy = pos.second - world_goal.second;
                new_states.push_back(new WaypointState(x, y, this->cost + maze.get_resolution(),
                                    sqrt(vx*vx+vy*vy), goal));
            }
        }
        return new_states;
    }
private:
    WaypointState(int x, int y, double cost, pair<int, int> parent, double heuristic, 
            pair<int, int> goal) :
        State(x, y, cost, parent, heuristic), goal(goal) {};

    pair<int, int> goal;
};

class Waypoint {
public:
    Waypoint(ros::NodeHandle n) : n(n),
        map_sub(n, "/map", 1), 
        odom_sub(n, "/ass1/odom", 1), 
        sync(ApproxPolicy(10), map_sub, odom_sub)
    {    
        //sync.registerCallback(boost::bind(&Waypoint::map_callback, this, _1, _2)); 
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);
    }

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og, 
            const nav_msgs::Odometry::ConstPtr &odom) {
        this->maze.set_occupancy_grid(*og);
        this->bot.update(odom);
    }

    Maze maze;
    Bot bot;

    ros::NodeHandle n;
    ros::Publisher movement_pub;
    
    message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<ApproxPolicy> sync;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n;

    Waypoint waypoint(n);

    ros::spin();

    return 0;
}
