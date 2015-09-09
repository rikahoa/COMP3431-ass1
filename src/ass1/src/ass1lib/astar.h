#ifndef _ASS1_ASTAR_H
#define _ASS1_ASTAR_H

#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include "maze.h"
#include "bot.h"

#define SAFE_PERCENT 60

using namespace std;

class State {
public:
    State(int x, int y, double cost, double heuristic) : 
        State(x, y, cost, make_pair(-1, -1), heuristic) {};
    virtual ~State() {};

    double get_cost() const { 
        return cost; 
    }

    double get_total_cost() const {
        return get_cost() + get_heuristic();
    }

    double get_heuristic() const {
        return heuristic;
    }

    virtual bool is_goal(const Maze& maze) const = 0;

    pair<int, int> get_position() const {
        return make_pair(x, y);
    }

    pair<int, int> get_parent() const {
        return parent;
    }

    friend bool operator<(const State &a, const State &b) {
        return a.get_total_cost() < b.get_total_cost();
    }
    
    virtual vector<State*> explore(const Maze& maze,
            std::function<bool(pair<int,int>)> check) const = 0; 
protected:
    int x, y;
    double cost;
    pair<int, int> parent;
    double heuristic;
    
    static const vector<pair<int, int>> DIRECTIONS;

    State(int x, int y, double cost, pair<int, int> parent, double heuristic) : 
        x(x), y(y), cost(cost), parent(parent), heuristic(heuristic) {};
};

class WaypointState : public State {
public:
    WaypointState(int x, int y, double cost, double heuristic, pair<int,int> goal, double accepted) : 
        WaypointState(x, y, cost, heuristic, make_pair(-1, -1), goal, accepted) {
        
    }

    virtual bool is_goal(const Maze& maze) const override {
        double vy = y - goal.second;
        double vx = x - goal.first;
        auto distance = sqrt(vy*vy+vx*vx) * maze.get_resolution();
        return distance < this->accepted;
    }

    virtual vector<State*> explore(const Maze& maze,
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        auto world_goal = maze.get_world_pos(this->goal);
        for (const auto &p : State::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;
            
            if (maze.get_data(this->x, this->y) < SAFE_PERCENT) {
                if (x >= 0 && x < maze.get_width() && 
                        y >= 0 && y < maze.get_height() && 
                        check(make_pair(x, y))) {
                    auto pos = maze.get_world_pos(make_pair(x, y)); 
                    double vx = pos.first - world_goal.first;
                    double vy = pos.second - world_goal.second;
                    new_states.push_back(new WaypointState(x, y, this->cost + maze.get_resolution(),
                                        sqrt(vx*vx+vy*vy), make_pair(this->x, this->y),
                                        goal, this->accepted));
                }
            }
        }
        return new_states;
    }
private:
    WaypointState(int x, int y, double cost, double heuristic, 
        pair<int, int> parent, pair<int, int> goal, double accepted) :
        State(x, y, cost, parent, heuristic), goal(goal), accepted(accepted) {
        
    };

    pair<int, int> goal;
    double accepted;
};

class ExplorationState : public State {
public:
    ExplorationState(int x, int y, double cost, Bot* bot) : 
        ExplorationState(x, y, cost, make_pair(-1, -1), bot) {};

    virtual bool is_goal(const Maze& maze) const override {
        return maze.get_data(this->x, this->y) == -1 &&
            this->bot->astar_okay(maze.get_world_pos(make_pair(this->x, this->y)));
    }

    virtual vector<State*> explore(const Maze& maze, 
            std::function<bool(pair<int,int>)> check) const override {
        vector<State*> new_states;
        for (const auto &p : State::DIRECTIONS) {
            int x = this->x + p.first;
            int y = this->y + p.second;
            
            // try move places.
            if (maze.get_data(this->x, this->y) < SAFE_PERCENT/* || 
                    this->bot->close_enough(maze.get_world_pos(make_pair(this->x, this->y)))*/) {
                if (x >= 0 && x < maze.get_width() && 
                        y >= 0 && y < maze.get_height() && 
                        check(make_pair(x, y))) {
                    new_states.push_back(
                            new ExplorationState(x, y, 
                                this->cost + maze.get_resolution(), this->get_position(), this->bot));
                }
            }
        }
        return new_states;
    }
private:
    ExplorationState(int x, int y, double cost, pair<int, int> parent, Bot* bot) :
        State(x, y, cost, parent, 0), bot(bot) {};
    Bot* bot;
};

const vector<pair<int,int>> State::DIRECTIONS = 
    vector<pair<int, int>>{make_pair(-1,0),make_pair(1,0),make_pair(0,-1),make_pair(0,1)};

// warning: initial_state gets deleted!
vector<pair<int, int>> search(
        const Maze& maze,
        State *initial_state);

#endif
