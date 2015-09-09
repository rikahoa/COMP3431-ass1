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

class Exploration {
public:
    Exploration(ros::NodeHandle n) : 
        started(false),
        spin(true),
        spin_yaw(-0.11),
        n(n),
        pnh("~")
    {
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);
        unstuck_pub = n.advertise<std_msgs::String>("/ass1/stuck", 1);
        beacons_sub = n.subscribe("ass1/beacons", 1, &Exploration::beacon_callback, this);
        odom_sub = n.subscribe("ass1/odom", 1, &Exploration::odom_callback, this);
        recalc_sub = n.subscribe("ass1/recalc", 1, &Exploration::recalc_callback, this);
        map_sub = n.subscribe("map", 1, &Exploration::map_callback, this);
        map_fatten_pub = n.advertise<nav_msgs::OccupancyGrid>("/ass1/map", 1);

        if (!pnh.getParam("fatten", fatten_value)) {
            ROS_INFO("Failed to get fatten param");
        } else {
            ROS_INFO("Got fatten param");
        }
        maze = Maze(fatten_value);
    }

private:
    void beacon_callback(const ass1::FoundBeacons::ConstPtr& msg) {
        ROS_INFO_STREAM("Beacons message on Exploration! Shutting down.");
        ros::shutdown();
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og) {
        this->maze.set_occupancy_grid(*og);
        // == publish for debugging...
        vector<pair<double,double>> extra;
        if (!path.empty()) {
            extra.push_back(path.front());
        }
        this->maze.rviz(map_fatten_pub, this->og_path, extra);
        // == remove when done
    }

    bool recalculate_astar() {
        // Find current position.
        auto og_pos = this->bot.get_og_pos(this->maze);
    
        ROS_INFO_STREAM("* ASTAR invoked.");
        // Do a A* to the nearest frontier
        vector<pair<int,int>> og_path;
        if (!started || this->bot.close_enough(this->target) || 
                maze.certain(this->og_target)) {
            og_path = search(this->maze, 
                new ExplorationState(og_pos.first, og_pos.second, 0, &this->bot));
        } else {
            ROS_INFO_STREAM("recalculating path to certain target...");
            og_path = search(this->maze, 
                new WaypointState(og_pos.first, og_pos.second, 0, 0, og_target, 0.5));
            if (og_path.empty()) {
                ROS_INFO_STREAM("giving up...can't get to that point...");
                og_path = search(this->maze, 
                    new ExplorationState(og_pos.first, og_pos.second, 0, &this->bot));
            }
        }
        // can't find path!
        if (og_path.empty()) {
            ROS_ERROR_STREAM("* No target found...");
            return false;
        }

        // Target the frontier in real position.
        this->og_target = og_path.back();
        this->og_path = og_path;
        this->path = this->maze.og_to_real_path(this->og_path);
        this->target = this->path.back();
        this->started = true;

        // == publish for debugging...
        vector<pair<double,double>> extra;
        if (!path.empty()) {
            extra.push_back(path.front());
        }
        this->maze.rviz(map_fatten_pub, this->og_path, extra);
        // == remove when done

        ROS_DEBUG_STREAM("* Let's find " << og_target.first << "," << og_target.second);
        return true;
    }

    void send_unstuck() {
        std_msgs::String result;
        result.data = "Bob";
        unstuck_pub.publish(result);
    }

    void recalc_callback(const std_msgs::String::ConstPtr &msg) {
        ROS_INFO_STREAM("Recalculate whores!");
        if (this->bot.valid() && this->maze.valid()) {
            recalculate_astar();
        }
    }

    void start_spin() {
        this->spin = true;
        this->spin_yaw = this->bot.get_yaw() - 0.11;
        // fix the angle
        this->spin_yaw -= static_cast<int>(this->spin_yaw / (2*PI)) * 2 * PI;
        // fix to spin the right way around
        if (this->spin_yaw > PI) {
            this->spin_yaw = -this->spin_yaw + PI;
        } else if (this->spin_yaw < -PI) {
            this->spin_yaw = -this->spin_yaw - PI;
        }
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
        this->bot.update(odom);

        if (this->maze.valid()) {
            geometry_msgs::TwistStamped move;
            move.header = odom->header;

            // check for spinning
            /*if (this->spin) {
                if (fabs(this->bot.get_yaw() - spin_yaw) > 0.1) {
                    ROS_INFO_STREAM("~~~~~~~~~~~ spin fat bastard! ~~~~~~~~~~");
                    this->bot.setup_spin(move.twist, fabs(this->bot.get_yaw() - spin_yaw));
                    movement_pub.publish(move);
                    return;
                }
                this->spin = false;
            }*/

            ROS_INFO_STREAM("~~~~~~~~~~~ move fat bastard! ~~~~~~~~~~");
            ROS_DEBUG_STREAM("maze getting " << og_target.first << "," << og_target.second 
                    << ":" << this->maze.get_data(og_target.first, og_target.second));
            
            // Continue while the goal is unknown.
            if (!started || path.empty() || this->bot.close_enough(path.back())) {
                if (!recalculate_astar()) {
                    send_unstuck();
                    return;
                }
            }

            // Populate until next path is found.
            while (!this->path.empty() && this->bot.close_enough(path.front())) {
                ROS_INFO_STREAM("close enough to " << path.front().first << "," << 
                        path.front().second << " ... popping");
                path.pop();
            }

            // Error checking
            if (path.empty()) {
                ROS_WARN_STREAM("Exploration path empty! Cannot move anywhere...");
                if (!recalculate_astar()) {
                    send_unstuck();
                }
                return;
            }

            ROS_DEBUG_STREAM("We need to know " << og_target.first << "," << og_target.second << 
                    " (world pos " << path.back().first << "," << path.back().second << ")");
            
            // Generate me a move message to target.
            this->bot.setup_movement(path.front(), move.twist);
            movement_pub.publish(move);
        }
    }

    bool started;
    bool spin;
    double spin_yaw;
    int fatten_value;
    Maze maze;
    Bot bot;

    ros::NodeHandle n;
    ros::NodeHandle pnh;

    ros::Publisher movement_pub;    
    ros::Publisher unstuck_pub;    
    ros::Publisher map_fatten_pub;
    ros::Subscriber map_sub;
    ros::Subscriber recalc_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber beacons_sub;

    pair<int, int> og_target;
    pair<double, double> target;
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
