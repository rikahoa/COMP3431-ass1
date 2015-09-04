#ifndef _ASS1_ASTAR_H
#define _ASS1_ASTAR_H

#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>
#include "maze.h"

using namespace std;

class State {
public:
    State(int x, int y, int cost, double heuristic) : 
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

    State(int x, int y, int cost, pair<int, int> parent, double heuristic) : 
        x(x), y(y), cost(cost), parent(parent), heuristic(heuristic) {};
};

const vector<pair<int,int>> State::DIRECTIONS = 
    vector<pair<int, int>>{make_pair(-1,0),make_pair(1,0),make_pair(0,-1),make_pair(0,1)};

// warning: initial_state gets deleted!
vector<pair<int, int>> search(
        const Maze& maze,
        State *initial_state);

#endif
