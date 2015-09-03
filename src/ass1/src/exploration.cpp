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
#include "geometry_msgs/TwistStamped.h"

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::OccupancyGrid, nav_msgs::Odometry> ApproxPolicy;

class ExplorationState : public State {
public:
    ExplorationState(int x, int y, int cost) : 
        ExplorationState(x, y, cost, make_pair(-1, -1)) {};

    virtual bool is_goal(const Maze& maze) const override {
        return maze.get_occupancy_grid().data[y * maze.get_occupancy_grid().info.height + x] == -1;
    }

    virtual vector<State*> explore(const Maze& maze, 
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        
        for (const auto &p : ExplorationState::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;
            if (x >= 0 && x < maze.get_occupancy_grid().info.width && 
                    y >= 0 && y < maze.get_occupancy_grid().info.height && 
                    check(make_pair(x, y)) &&
                    maze.get_occupancy_grid().
                        data[y * maze.get_occupancy_grid().info.height + x] <= 0) {
                new_states.push_back(
                        new ExplorationState(x, y, this->get_cost() + 1, 
                            this->get_position()));
            }
        }
        return new_states;
    }
private:
    // for speed ups
    static const vector<pair<int, int>> DIRECTIONS;

    ExplorationState(int x, int y, int cost, pair<int, int> parent) :
        State(x, y, cost, parent, 0) {};
};

const vector<pair<int,int>> ExplorationState::DIRECTIONS = 
    vector<pair<int, int>>{make_pair(-1,0),make_pair(1,0),make_pair(0,-1),make_pair(0,1)};

class Exploration {
public:
    Exploration(ros::NodeHandle n) : n(n),
        map_sub(n, "/map", 1), 
        odom_sub(n, "/odom", 1), 
        sync(ApproxPolicy(10), map_sub, odom_sub)   
    {
        sync.registerCallback(boost::bind(&Exploration::map_callback, this, _1, _2)); 
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);

    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og, 
            const nav_msgs::Odometry::ConstPtr &odom) {
        this->maze.set_occupancy_grid(*og);
        this->bot.update(odom);

        // TODO: put coordinates in
        auto path = search(this->maze, new ExplorationState(0, 0, 0));

        /*auto ogp = this->bot.get_occupancy_grid_coord(og->info.resolution);
        ROS_INFO_STREAM("point: " << ogp.first << "," << ogp.second << ":" << 
                maze.get_data(ogp.first, ogp.second));*/
    }
private:
    Maze maze;
    Bot bot;
    pair<double, double> target;

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
