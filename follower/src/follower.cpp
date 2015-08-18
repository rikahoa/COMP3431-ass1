#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


ros::Publisher  pub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!msg->ranges.empty()) {
	int minindex = -1;
	float min = FLT_MAX;

	for(int i = 0; i < msg->ranges.size(); ++i) {
		if(msg->ranges[i] < min) {
			min = msg->ranges[i];
			minindex = i;
		}
	}

        ROS_INFO("min range is %f", min);
	ROS_INFO("min index is %d", minindex);
	float newangle = msg->angle_min + minindex*msg->angle_increment;
	ROS_INFO("new angle %f", newangle);

	if( newangle > 0.1 || newangle < -0.1) {
	//Cheat way turn left  or right by the angle we want  within a threhsold
		geometry_msgs::Twist move;


		move.linear.x = 0;
		move.linear.y = 0;
		move.linear.z = 0;

		move.angular.x = 0;
		move.angular.y = 0;
		move.angular.z = newangle;

		pub.publish(move);
	}
	
    }
}



int main(int argc, char **argv)
{


  ros::init(argc, argv, "twoway");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("scan", 1, laserCallback);
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);

  ros::spin();

  return 0;
}
