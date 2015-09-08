#include <ros/ros.h>

#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, sensor_msgs::LaserScan> ApproxTwistLaserPolicy;
        
class Movement {
public:
    Movement(ros::NodeHandle n) : n(n),movement_sub(n, "/ass1/movement", 1),laser_sub(n, "/scan", 1), sync(ApproxTwistLaserPolicy(10), movement_sub, laser_sub)   {
        navi_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
        recalc_pub = n.advertise<std_msgs::String>("/ass1/recalc", 1);
        sync.registerCallback(boost::bind(&Movement::movement_and_laser_callback, this, _1, _2));
    }
private:
    ros::NodeHandle n;
    ros::Publisher navi_pub;
    ros::Publisher recalc_pub;

    message_filters::Subscriber<geometry_msgs::TwistStamped> movement_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

    message_filters::Synchronizer<ApproxTwistLaserPolicy> sync;
            
    void movement_and_laser_callback(const geometry_msgs::TwistStamped::ConstPtr &twist_stamped, 
            const sensor_msgs::LaserScan::ConstPtr &laser_scan) {
        bool safe = false;

        if (twist_stamped->twist.linear.x == 0) {
            safe = true;
        } else {
            safe = true;          
            for (const auto& range : laser_scan->ranges) {
                if (range < 0.18) {
                    safe = false;
                    break;
                }
            }
        }

        if (safe) {
             // Publish messages
             ROS_DEBUG_STREAM("MOVEMENT: x = " << twist_stamped->twist.linear.x << 
                ", angle z = " << twist_stamped->twist.angular.z);
             navi_pub.publish(twist_stamped->twist);
        } else {
             ROS_ERROR_STREAM("Cannot Move x = " << twist_stamped->twist.linear.x << 
                    ", angle z = " << twist_stamped->twist.angular.z);
            
             // signal to recalculate
             std_msgs::String msg;
             recalc_pub.publish(msg);
        }
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
