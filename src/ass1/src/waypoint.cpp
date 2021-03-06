#include "ros/ros.h"
#include <utility>
#include "ass1/FoundBeacons.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/String.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "ass1lib/astar.h"
#include "ass1lib/maze.h"
#include "ass1lib/bot.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::OccupancyGrid, nav_msgs::Odometry> ApproxPolicy;


class Waypoint {
public:
    Waypoint(ros::NodeHandle n) : n(n), pnh("~"), started(false)
    {    
        beacons_sub = n.subscribe("ass1/beacons", 1, &Waypoint::beacon_callback, this);
        unstuck_pub = n.advertise<std_msgs::String>("/ass1/stuck", 1);
        map_sub = n.subscribe("map", 1, &Waypoint::map_callback, this);
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
        ROS_INFO_STREAM("WAYPOINT BEGIN: Beacons found! Dora the Explorer is now Dora the Waypoint.");

        // start this shizzle up!
        movement_pub = n.advertise<geometry_msgs::TwistStamped>("/ass1/movement", 1);
        odom_sub = n.subscribe("ass1/odom", 1, &Waypoint::odom_callback, this);
        recalc_sub = n.subscribe("ass1/recalc", 1, &Waypoint::recalc_callback, this);
        
        to_visit = queue<pair<double,double>>();
        for (auto it = msg->positions.begin(); it != msg->positions.end(); ++it) {
            ROS_INFO_STREAM("WAYPOINT: beacon: " << it->x << "," << it->y);
            to_visit.push(make_pair(it->x, it->y));
        }
    }

    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &og) {
        this->maze.set_occupancy_grid(*og);
        if (started) {
            this->maze.rviz(map_fatten_pub, this->og_path, 
                    vector<pair<double,double>>{this->to_visit.front()});
        }
    }

    bool recalculate_astar() {
        // recalculate position
        auto og_pos = this->bot.get_og_pos(this->maze);
        auto og_path = search(this->maze, 
                new WaypointState(og_pos.first, og_pos.second, 0, 0, 
                    this->maze.get_og_pos(to_visit.front()), 0.25));
        if (og_path.empty()) {
            ROS_ERROR_STREAM("WAYPOINT: * No Target found...");
            return false;
        }

        this->og_path = og_path;
        this->path = this->maze.og_to_real_path(og_path);
        this->path.push(to_visit.front());

        // ==== debugging
        vector<pair<double,double>> extra{this->to_visit.front()};
        if (!path.empty()) {
            extra.push_back(path.front());
        }
        this->maze.rviz(map_fatten_pub, this->og_path, extra);
        // ===== end
        
        // push path to the end
        this->started = true;
        return true;
    }
    
    void recalc_callback(const std_msgs::String::ConstPtr &msg) {
        ROS_INFO_STREAM("WAYPOINT: Recalculate whores!");
        recalculate_astar();
    }

    void send_unstuck() {
        std_msgs::String result;
        result.data = "Bob";
        unstuck_pub.publish(result);
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom) {
        this->bot.update(odom);

        // Visited them all! ignore all others.
        if (this->to_visit.empty()) {
            return;
        }

        if (this->maze.valid()) {
            while (!started || this->path.empty() || 
                this->bot.close_enough(to_visit.front())) 
            {
                if (started && this->bot.close_enough(to_visit.front())) {
                    ROS_INFO_STREAM("WAYPOINT: found" << to_visit.front().first << 
                            "," << to_visit.front().second );
                    this->to_visit.pop();
                    if (this->to_visit.empty()) {
                        ROS_INFO_STREAM("WAYPOINT: targets found. Shutting down...");
                        ros::shutdown();
                        return;
                    }
                }
                if (!recalculate_astar()) {
                    send_unstuck();
                    return;
                }
            }

            // Populate until next path is found.
            while (!this->path.empty() && this->bot.close_enough(path.front())) {
                ROS_INFO_STREAM("WAYPOINT: close enough to " << path.front().first << "," << 
                        path.front().second << " ... popping");
                path.pop();
            }
            
            if (path.empty()) {
                ROS_ERROR_STREAM("WAYPOINT: path empty! Cannot move anywhere...");
                if (!recalculate_astar()) {
                    send_unstuck();
                }
                return;
            }

            // Generate me a move message to target.
            geometry_msgs::TwistStamped move;
            move.header = odom->header;
            this->bot.setup_movement(path.front(), move.twist);
            movement_pub.publish(move);
        }
    }

    Maze maze;
    Bot bot;

    ros::NodeHandle n;
    ros::NodeHandle pnh;
    
    ros::Publisher movement_pub;
    ros::Publisher unstuck_pub;    
    ros::Publisher map_fatten_pub;
    
    ros::Subscriber beacons_sub;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber recalc_sub;
    
    bool started;
    int fatten_value;

    vector<pair<int, int>> og_path;
    queue<pair<double, double>> to_visit;
    queue<pair<double, double>> path;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n;

    Waypoint waypoint(n);

    ros::spin();

    return 0;
}
