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
    Exploration(ros::NodeHandle n) : n(n), dis(-2,2), gen(rd()) /*,
        map_sub(n, "/map", 1), 
        odom_sub(n, "/ass1/odom", 1), 
        sync(ApproxPolicy(10), map_sub, odom_sub)   */
    {
        /*
        sync.registerCallback(boost::bind(&Exploration::map_callback, this, _1, _2)); 
        */
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);

        // set odom and make random target
        odom_sub = n.subscribe("ass1/odom", 1, &Exploration::odom_callback, this);
        target = make_pair(1, 1);
        
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
        this->bot.update(odom);

        // Generate me a message.
        geometry_msgs::TwistStamped move;
        move.header = odom->header;
        move.twist.linear.x = move.twist.linear.y = move.twist.linear.z = 0;
        move.twist.angular.x = move.twist.angular.y = move.twist.angular.z = 0;

        auto displacement = this->bot.get_displacement(target.first, target.second);
        
        // We have reached our destination - make a new one.
        while (displacement.first < 0.1) {
            // generate random location
            ROS_INFO_STREAM("reached target " << target.first << "," << target.second); 
            target = make_pair(dis(gen), dis(gen));
            displacement = this->bot.get_displacement(target.first, target.second);
        }

        ROS_INFO_STREAM("target of " << target.first << "," << target.second);
        ROS_INFO_STREAM("we are at " << this->bot.get_position().first << "," 
                << this->bot.get_position().second);
        ROS_INFO_STREAM("angle change of " << displacement.second << " required.");
        ROS_INFO_STREAM("displacement from target is " << displacement.first);

        if (fabs(displacement.second) > 0.05) {
            // Rotation required.
            move.twist.angular.z = displacement.second;
        } else {
            // Otherwise, move towards our destination.
            move.twist.linear.x = displacement.first;
        }

        movement_pub.publish(move);
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og, 
            const nav_msgs::Odometry::ConstPtr &odom) {
        this->maze.set_occupancy_grid(*og);
        this->bot.update(odom);

        auto og_pos = this->bot.get_og_coord(this->maze);
        ROS_INFO_STREAM("point: " << og_pos.first << "," << og_pos.second << ":" <<
                maze.get_data(og_pos.first, og_pos.second));
        
        // Do a A* to the nearest frontier
        //auto path = search(this->maze, new ExplorationState(og_pos.x, og_pos.y, 0));

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
    
    /*message_filters::Subscriber<nav_msgs::OccupancyGrid> map_sub;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
    message_filters::Synchronizer<ApproxPolicy> sync;*/

    ros::Subscriber odom_sub;
    std::uniform_real_distribution<> dis;
    std::random_device rd;
    std::mt19937 gen;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    Exploration exploration(n);
    ros::spin();

    return 0;
}
