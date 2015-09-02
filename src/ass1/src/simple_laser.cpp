#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <algorithm>
#include <cmath>

constexpr int FIELD_OF_VIEW_DEG = 55;

constexpr float PI = acos(-1);
constexpr float FIELD_OF_VIEW = FIELD_OF_VIEW_DEG*2*PI/360;

class SimpleLaser {
public:
    
    SimpleLaser(ros::NodeHandle n) : n(n) {
        simple_laser_pub = n.advertise<sensor_msgs::LaserScan>("/ass1/scan", 1);
        laser_sub = n.subscribe("/scan", 1, &SimpleLaser::laserCallback, this);
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& oldLaserScan) {
        sensor_msgs::LaserScan newLaserScan;
        
        newLaserScan.header = oldLaserScan->header;
        
        newLaserScan.angle_min = -FIELD_OF_VIEW/2;        
        newLaserScan.angle_max = FIELD_OF_VIEW/2;
        
        newLaserScan.angle_increment = oldLaserScan->angle_increment;  
        newLaserScan.time_increment = oldLaserScan->time_increment;  
        newLaserScan.scan_time = oldLaserScan->scan_time;
        newLaserScan.range_min = oldLaserScan->range_min;
        newLaserScan.range_max = oldLaserScan->range_max;

        int counts = FIELD_OF_VIEW/newLaserScan.angle_increment;
        int start_index = (newLaserScan.angle_min - oldLaserScan->angle_min)/newLaserScan.angle_increment;
        
        std::copy(oldLaserScan->ranges.begin() + start_index, oldLaserScan->ranges.begin() + start_index + counts, std::back_inserter(newLaserScan.ranges));
        std::copy(oldLaserScan->intensities.begin() + start_index, oldLaserScan->intensities.begin() + start_index + counts, back_inserter(newLaserScan.intensities));
        
        simple_laser_pub.publish(newLaserScan);
    }

private:
    ros::NodeHandle n;
    ros::Publisher simple_laser_pub;
    ros::Subscriber laser_sub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "simpleLaser");
    ros::NodeHandle n;

    SimpleLaser simpleLaser(n);
    ROS_INFO("Simple Laser setup successfully.");

    ros::spin();

    return 0;
}
