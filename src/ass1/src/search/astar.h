#include <iostream>
#include <queue>
#include <vector>
#include <algorithm>

using namespace std;

class State {
public:
    State(int x, int y, int cost) : 
        State(x, y, cost, make_pair(-1, -1)) {};
    virtual ~State() {};
    State(const State &s) : x(s.x), y(s.y), cost(s.cost), parent(s.parent) {};
    State(State &&s) : x(s.x), y(s.y), cost(s.cost), parent(s.parent) {};
    State& operator=(const State& s) { 
        this->x = s.x; 
        this->y = s.y; 
        this->cost = s.cost; 
        this->parent = s.parent;
        return *this;
    }
    State& operator=(State&& s) { 
        this->x = s.x; 
        this->y = s.y; 
        this->cost = s.cost;
        this->parent = s.parent;
        return *this;
    }

    double get_cost() const { 
        return cost; 
    }

    double get_total_cost() const {
        return get_cost() + get_heuristic();
    }

    virtual double get_heuristic() const = 0;

    virtual bool is_goal(const vector<vector<int>> &map, int xmax, int ymax) const = 0;

    pair<int, int> get_position() const {
        return make_pair(x, y);
    }

    pair<int, int> get_parent() const {
        return parent;
    }

    friend bool operator<(const State &a, const State &b) {
        return a.get_total_cost() < b.get_total_cost();
    }
    
    virtual vector<State*> explore(const vector<vector<int>> &map, int xmax, int ymax, 
            std::function<bool(pair<int,int>)> check) const = 0; 
protected:
    int x, y;
    double cost;
    pair<int, int> parent;

    State(int x, int y, int cost, pair<int, int> parent) : 
        x(x), y(y), cost(cost), parent(parent) {};
};

// warning: initial_state gets deleted!
vector<pair<int, int>> search(const vector<vector<int>> &map, 
        int xmax, int ymax, 
        State *initial_state); 
