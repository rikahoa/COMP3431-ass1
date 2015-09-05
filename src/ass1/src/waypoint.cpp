#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ass1lib/astar.h"
#include <utility>

using namespace std;

class WaypointState : public State {
public:
    WaypointState(int x, int y, double cost, double heuristic, pair<int,int> goal) : 
        WaypointState(x, y, cost, make_pair(-1, -1), heuristic, goal) {
        
    }

    virtual bool is_goal(const Maze& maze) const override {
        return x == goal.first && y == goal.second;
    }

    virtual vector<State*> explore(const Maze& maze,
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        auto world_goal = maze.get_world_pos(this->goal);
        for (const auto &p : State::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;

            if (x >= 0 && x < maze.get_width() && 
                    y >= 0 && y < maze.get_height() && 
                    check(make_pair(x, y)) &&
                    maze.get_data(x, y) <= 0) {
                auto pos = maze.get_world_pos(make_pair(x, y)); 
                double vx = pos.first - world_goal.first;
                double vy = pos.second - world_goal.second;
                new_states.push_back(new WaypointState(x, y, this->cost + maze.get_resolution(),
                                    sqrt(vx*vx+vy*vy), goal));
            }
        }
        return new_states;
    }
private:
    WaypointState(int x, int y, double cost, pair<int, int> parent, double heuristic, 
            pair<int, int> goal) :
        State(x, y, cost, parent, heuristic), goal(goal) {};

    pair<int, int> goal;
};

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
