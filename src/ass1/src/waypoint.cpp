#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

/*class WaypointState : public State {
public:
    WaypointState(int x, int y, double cost, pair<int,int> goal) : 
        WaypointState(x, y, cost, make_pair(-1, -1), goal) {
        
    }
private:
    WaypointState(int x, int y, double cost, pair<int, int> parent, pair<int, int> goal) :
        State(x, y, cost, parent, 0), goal(goal) {
        
    };

    pair<int, int> goal;
}*/

class Waypoint {
public:
    Waypoint(ros::NodeHandle n) : n(n) {
    }

private:
    ros::NodeHandle n;
    ros::Publisher movement_pub;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "waypoint");
    ros::NodeHandle n;

    Waypoint waypoint(n);

    ros::spin();

    return 0;
}
