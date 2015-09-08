#include <ros/ros.h>

#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

typedef message_filters::sync_policies::ApproximateTime
    <geometry_msgs::TwistStamped, sensor_msgs::LaserScan> ApproxTwistLaserPolicy;

constexpr float PI = acos(-1);
        
class Movement {
public:
    Movement(ros::NodeHandle n) : n(n), pnh("~"),
        movement_sub(n, "/ass1/movement", 1),
        laser_sub(n, "/scan", 1), 
        unstuck_sub(n, "/ass1/unstuck", 1),
        sync(ApproxTwistLaserPolicy(10), movement_sub, laser_sub),
        unstuck_sync(ApproxTwistLaserPolicy(100), unstuck_sub, laser_sub)
    {
        navi_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
        recalc_pub = n.advertise<std_msgs::String>("/ass1/recalc", 1);
        sync.registerCallback(boost::bind(&Movement::movement_and_laser_callback, this, _1, _2));
        unstuck_sync.registerCallback(boost::bind(&Movement::unstuck_callback, this, _1, _2));
        
        if (pnh.getParam("safe_range", safe_range)) { ROS_INFO("Got safe_range param"); } else { ROS_ERROR("Failed to get param 'safe_range'");}
        if (pnh.getParam("unstuck_x_movement", unstuck_x_movement)) { ROS_INFO("Got unstuck_x_movement param"); } else { ROS_ERROR("Failed to get param 'unstuck_x_movement'");}
        if (pnh.getParam("unstuck_angle_threshold", unstuck_angle_threshold)) { ROS_INFO("Got unstuck_angle_threshold param"); } else { ROS_ERROR("Failed to get param 'unstuck_angle_threshold'");}
        if (pnh.getParam("unstuck_angle_multiplier", unstuck_angle_multiplier)) { ROS_INFO("Got unstuck_angle_multiplier param"); } else { ROS_ERROR("Failed to get param 'unstuck_angle_multiplier'");}
    }
private:
    ros::NodeHandle n;
    ros::NodeHandle pnh;
    ros::Publisher navi_pub;
    ros::Publisher recalc_pub;

    message_filters::Subscriber<geometry_msgs::TwistStamped> movement_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    message_filters::Subscriber<geometry_msgs::TwistStamped> unstuck_sub;

    message_filters::Synchronizer<ApproxTwistLaserPolicy> sync;
    message_filters::Synchronizer<ApproxTwistLaserPolicy> unstuck_sync;
    
    //PARAMS
    double safe_range;
    double unstuck_x_movement;
    double unstuck_angle_threshold;
    double unstuck_angle_multiplier;
    
            
    void movement_and_laser_callback(const geometry_msgs::TwistStamped::ConstPtr &twist_stamped, 
            const sensor_msgs::LaserScan::ConstPtr &laser_scan) {
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
             
             // do some wall following...
             auto twist = twist_stamped->twist;
             unstuck(laser_scan);

             // signal to recalculate
             std_msgs::String msg;
             recalc_pub.publish(msg);
        }
    }

    void unstuck_callback(const geometry_msgs::TwistStamped::ConstPtr &twist_stamped, 
            const sensor_msgs::LaserScan::ConstPtr &laser_scan) {
        ROS_ERROR_STREAM("unstuck callback!");
        unstuck(laser_scan);
    }

    void unstuck(const sensor_msgs::LaserScan::ConstPtr &msg) {
        auto it = std::min_element(msg->ranges.begin(), msg->ranges.end());
        if (it == msg->ranges.end()) {
            ROS_INFO("No data received.");
            return;
        }
        auto minindex = std::distance(msg->ranges.begin(), it);

        float minangle = msg->angle_min + minindex * msg->angle_increment;

        
        ROS_DEBUG_STREAM("min=" << *it << ",minindex=" << 
                minindex << ",minangle=" << minangle);

        geometry_msgs::Twist move;
        move.linear.y = 0;        
        move.linear.x = this->unstuck_x_movement;
        move.linear.z = 0;

        move.angular.x = 0;
        move.angular.y = 0;        
        move.angular.z = 0;
        
        if (fabs(minangle) > this->unstuck_angle_threshold) {
            move.angular.z = -this->unstuck_angle_multiplier*minangle;
        } 
             
        navi_pub.publish(move);
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
