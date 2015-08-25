#include "ros/ros.h"

class Beacon {
    private:
        double x, y;
        // COLOURS
};

class BeaconFinder {
    public:
        
        BeaconFinder(ros::NodeHandle n) : n(n) {
            // location_pub = n.advertise<??>("/ass1/beacons", 1);
        }

    private:
        ros::NodeHandle n;
        ros::Publisher location_pub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "beacon_finder");
    ros::NodeHandle n;

    BeaconFinder beacon_finder(n);

    ros::spin();

    return 0;
}
