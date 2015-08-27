#include "ros/ros.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>

class Movement {
    public:
        Movement(ros::NodeHandle n) : n(n) {
            navi_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  	
            message_filters::Subscriber<geometry_msgs::TwistStamped> movement_sub(n, "/ass1/movement", 1);
  	    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(n, "/scan", 1);
	    //TODO Make it an approximate time synchronizer
	    message_filters::TimeSynchronizer<geometry_msgs::TwistStamped, sensor_msgs::LaserScan> sync(movement_sub, laser_sub, 10);
	    sync.registerCallback(boost::bind(&Movement::movement_and_laser_callback, this, _1, _2));
	}
    private:
        ros::NodeHandle n;
        ros::Publisher navi_pub;

        void movement_and_laser_callback(const geometry_msgs::TwistStampedConstPtr &twistStamped, const sensor_msgs::LaserScanConstPtr &laserScan) {
            ROS_DEBUG_STREAM("Moving x = " << twistStamped->twist.linear.x << ", angle z = " << twistStamped->twist.angular.z);
	    //TODO Check with laser to see we wont crash before we move

            navi_pub.publish(twistStamped->twist);
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
