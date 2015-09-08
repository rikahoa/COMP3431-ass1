#include <ros/ros.h>

#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

#include <algorithm>

typedef message_filters::sync_policies::ApproximateTime
    <geometry_msgs::TwistStamped, sensor_msgs::LaserScan> ApproxTwistLaserPolicy;

constexpr float PI = acos(-1);
        
class Movement {
public:
    Movement(ros::NodeHandle n) : n(n), pnh("~"),
        movement_sub(n, "/ass1/movement", 1),
        laser_sub(n, "/scan", 1), 
        sync(ApproxTwistLaserPolicy(10), movement_sub, laser_sub),
        stuck(true)

    {
        navi_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
        recalc_pub = n.advertise<std_msgs::String>("/ass1/recalc", 1);
        sync.registerCallback(boost::bind(&Movement::movement_and_laser_callback, this, _1, _2));
        
        stuck_sub = n.subscribe("ass1/stuck", 1, &Movement::stuck_callback, this);
        laser_unstuck_sub = n.subscribe("/scan", 1, &Movement::laser_unstuck_callback, this);
        
        if (pnh.getParam("safe_range", safe_range)) { 
            ROS_INFO("Got safe_range param"); 
        } else { 
            ROS_ERROR("Failed to get param 'safe_range'");
        }

        if (pnh.getParam("unstuck_x_movement", unstuck_x_movement)) { 
            ROS_INFO("Got unstuck_x_movement param"); 
        } else { 
            ROS_ERROR("Failed to get param 'unstuck_x_movement'");
        }
        if (pnh.getParam("unstuck_angle_threshold", unstuck_angle_threshold)) { 
            ROS_INFO("Got unstuck_angle_threshold param"); 
        } else { 
            ROS_ERROR("Failed to get param 'unstuck_angle_threshold'");
        }
        if (pnh.getParam("unstuck_angle_multiplier", unstuck_angle_multiplier)) { 
            ROS_INFO("Got unstuck_angle_multiplier param"); 
        } else { 
            ROS_ERROR("Failed to get param 'unstuck_angle_multiplier'");
        }
    }
private:
    ros::NodeHandle n;
    ros::NodeHandle pnh;
    ros::Publisher navi_pub;
    ros::Publisher recalc_pub;

    message_filters::Subscriber<geometry_msgs::TwistStamped> movement_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    message_filters::Synchronizer<ApproxTwistLaserPolicy> sync;

    ros::Subscriber stuck_sub;
    ros::Subscriber laser_unstuck_sub;
    
    //PARAMS
    double safe_range;
    double unstuck_x_movement;
    double unstuck_angle_threshold;
    double unstuck_angle_multiplier;
    
    bool stuck;
    
            
    void movement_and_laser_callback(const geometry_msgs::TwistStamped::ConstPtr &twist_stamped, 
            const sensor_msgs::LaserScan::ConstPtr &laser_scan) { 
        if (!this->stuck) {
            bool safe = false;

            if (twist_stamped->twist.linear.x == 0) {
                if (twist_stamped->twist.angular.z != 0) {
                    safe = true;
                }
            } else {
                safe = true;          
                for (const auto& range : laser_scan->ranges) {
                    if (range < this->safe_range) {
                        safe = false;
                        break;
                    }
                }
            }

            if (safe) {
                 // Publish messages
                 ROS_INFO_STREAM("MOVEMENT: x = " << twist_stamped->twist.linear.x << 
                    ", angle z = " << twist_stamped->twist.angular.z);
                 navi_pub.publish(twist_stamped->twist);
            } else {
                 ROS_ERROR_STREAM("Cannot Move x = " << twist_stamped->twist.linear.x << 
                        ", angle z = " << twist_stamped->twist.angular.z);
                 
                 this->stuck=true;
            }
        }
    }

    void stuck_callback(const std_msgs::String::ConstPtr &msg) {
        ROS_WARN_STREAM("unstuck callback!");
        this->stuck = true;
    }

    void laser_unstuck_callback(const sensor_msgs::LaserScan::ConstPtr &laser) {
        if (safe(laser, this->safe_range)) {
            ROS_INFO("Laser Safe");
            if (this->stuck) {
                ROS_INFO("UNSTUCK: NOT STUCK ASKING RECALC");
                this->stuck = false;
                
                // signal to recalculate
                std_msgs::String msg;
                recalc_pub.publish(msg);
             }
        } else {
            ROS_INFO("Laser unsafe");
            this->stuck = true;
            ROS_INFO("UNSTUCK: STILL STUCK");
            auto it = std::min_element(laser->ranges.begin(), laser->ranges.end());
            auto minindex = std::distance(laser->ranges.begin(), it);

            float minangle = laser->angle_min + minindex * laser->angle_increment;

            ROS_INFO_STREAM("min=" << *it << ",minindex=" << minindex << ",minangle=" << minangle);

            geometry_msgs::Twist move;
            move.linear.y = 0;        
            move.linear.x = 0;
            move.linear.z = 0;

            move.angular.x = 0;
            move.angular.y = 0;        
            move.angular.z = 0;
            
            if (fabs(minangle) > this->unstuck_angle_threshold) {
                move.angular.z = std::max(-0.6, 
                        std::min(0.6, minangle*this->unstuck_angle_multiplier));
            } else {
                move.linear.x = this->unstuck_x_movement;
            }
                
            ROS_INFO_STREAM("UNSTUCK: moving " << move.linear.x << "," << move.angular.z << "...");
            navi_pub.publish(move); 
        }

    }
    
    static bool safe(const sensor_msgs::LaserScan::ConstPtr &laser, double safe_range) {
        auto it = std::min_element(laser->ranges.begin(), laser->ranges.end());
        return *it > safe_range;
    }
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "movement");
    ros::NodeHandle n;
    
    Movement movement(n);
    ROS_INFO("Movement setup successfully.");

    ros::spin();

    return 0;
}
