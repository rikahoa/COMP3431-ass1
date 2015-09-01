#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>


class Movement {
public:
    Movement(ros::NodeHandle n) : n(n) {
        navi_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

        message_filters::Subscriber<geometry_msgs::TwistStamped> movement_sub(n, "/ass1/movement", 1);
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(n, "/scan", 1);
        
        typedef message_filters::sync_policies::
            ApproximateTime<geometry_msgs::TwistStamped, sensor_msgs::LaserScan> 
            ApproxTwistLaserPolicy;

        message_filters::Synchronizer<ApproxTwistLaserPolicy> 
            sync(ApproxTwistLaserPolicy(5), movement_sub, laser_sub);
        sync.registerCallback(boost::bind(&Movement::movement_and_laser_callback, this, _1, _2));
    }
private:
    ros::NodeHandle n;
    ros::Publisher navi_pub;

    void movement_and_laser_callback(const geometry_msgs::TwistStampedConstPtr &twistStamped, 
            const sensor_msgs::LaserScanConstPtr &laserScan) {

        bool safe = false;

        if (twistStamped->twist.angular.z < laserScan->angle_min || 
                twistStamped->twist.angular.z > laserScan->angle_max) {
            safe = true;
        } else {
            double angle_delta = twistStamped->twist.angular.z - laserScan->angle_min;
            int delta = angle_delta / laserScan->angle_increment;
            
            if (laserScan->ranges[delta] < 0.1) {
                safe = true;            
            }
        }

        if (safe) {
             ROS_WARN_STREAM("Cannot move: x = " << twistStamped->twist.linear.x << 
                ", angle z = " << twistStamped->twist.angular.z);
             navi_pub.publish(twistStamped->twist);
        } else {
            ROS_DEBUG_STREAM("Moving x = " << twistStamped->twist.linear.x << 
                    ", angle z = " << twistStamped->twist.angular.z);
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
