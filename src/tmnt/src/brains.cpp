#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class Brains {
  public: 
	ros::NodeHandle n;
	ros::Publisher movementPub;
	ros::Subscriber laserNavSub;
	
	void laserNavigationCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

	};
	
	Brains(ros::NodeHandle node) {
	  n = node;
	  laserNavSub = n.subscribe("/scan", 1000, &Brains::laserNavigationCallback, this);
		movementPub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1000);
	}

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "brains");
  ros::NodeHandle n;
 
  Brains b  = Brains(n);
  
  ros::spin();

  return 0;
}
