#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "search/astar.h"

class ExplorationState : public State {
public:
    ExplorationState(int x, int y, int cost) : ExplorationState(x, y, cost, make_pair(-1, -1)) {};

    virtual double get_heuristic() const override {
        return 0;
    }
    
    virtual bool is_goal(const vector<vector<int>> &map, int xmax, int ymax) const override {
        return x == xmax - 1 && y == 0;
    }

    virtual vector<State*> explore(const vector<vector<int>> &map, int xmax, int ymax, 
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        
        for (const auto &p : ExplorationState::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;
            if (x >= 0 && x < xmax && y >= 0 && y < ymax && check(make_pair(x, y))) {
                new_states.push_back(
                        new ExplorationState(x, y, this->get_cost() + map[y][x], 
                            this->get_position()));
            }
        }
        return new_states;
    }
private:
    // for speed ups
    static const vector<pair<int, int>> DIRECTIONS;

    ExplorationState(int x, int y, int cost, pair<int, int> parent) :
        State(x, y, cost, parent) {};
};

const vector<pair<int,int>> ExplorationState::DIRECTIONS = 
    vector<pair<int, int>>{make_pair(-1,0),make_pair(1,0),make_pair(0,-1),make_pair(0,1)};

class Exploration {
public:
    Exploration(ros::NodeHandle n) : n(n) {
        movement_pub = n.advertise<geometry_msgs::Twist>("/ass1/movement", 1);
    }

private:
    ros::NodeHandle n;
    ros::Publisher movement_pub;
};

int main(int argc, char *argv[]) {
    vector<vector<int>> map;

    map.push_back(vector<int>{12,244,67,1});
    map.push_back(vector<int>{5,104,42,999});
    map.push_back(vector<int>{0,10,1,2});

    auto path = search(map, map[0].size(), map.size(), new ExplorationState(0, 0, 0));
    
    for (const auto &coord : path) {
        cout << "(" << coord.first << "," << coord.second << ")" << endl;
    }

    ros::init(argc, argv, "exploration");
    ros::NodeHandle n;

    Exploration exploration(n);

    ros::spin();

    return 0;
}
