#include <iostream>
#include <queue>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

class State {
    public:

        State(int x, int y, int cost, pair<int, int> parent) : 
            x(x), y(y), cost(cost), parent(parent) {};
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
        
        virtual vector<State*> explore(const vector<vector<int>> &map, int xmax, int ymax) const = 0; 
    protected:
        int x, y;
        double cost;
        pair<int, int> parent;
};

class ExplorationState : public State {
    public:
        ExplorationState(int x, int y, int cost, pair<int, int> parent) : 
            State(x, y, cost, parent) {};

        virtual double get_heuristic() const override {
            return 0;
        }
        
        virtual bool is_goal(const vector<vector<int>> &map, int xmax, int ymax) const override {
            return x == static_cast<int>(map.size()) - 1 && y == 0;
        }

        virtual vector<State*> explore(const vector<vector<int>> &map, int xmax, int ymax) const override {
            vector<State*> new_states;
            
            for (const auto &p : vector<pair<int, int>>{make_pair(-1,0),make_pair(1,0),make_pair(0,-1),make_pair(0,1)}) {
                int x = this->x + p.first;
                int y = this->y + p.second;
                if (x >= 0 && x < xmax && y >= 0 && y < ymax) {
                    new_states.push_back(new ExplorationState(x, y, this->get_cost() + map[y][x], this->get_position()));
                }
            }
            return new_states;
        }

};

class WaypointState : public State {
    public:
        WaypointState(int x, int y, int cost, pair<int, int> parent) : 
            State(x, y, cost, parent) {};
};


vector<pair<int, int>> 
search(const vector<vector<int>> &map, int xmax, int ymax, int xstart, int ystart) {
    class PairHash{
    public:
        size_t operator()(const pair<int, int> &k) const {
            return k.first * 10000 + k.second;
        }
    };

    struct PairEquals : binary_function<const pair<int,int>&, const pair<int,int>&, bool> {
        result_type operator()( first_argument_type lhs, second_argument_type rhs ) const {
            return (lhs.first == rhs.first) && (lhs.second == rhs.second);
        }
    };

    struct StatePtrEquals {
        bool operator()(const State* lhs, const State* rhs) {
            return *lhs < *rhs;
        }
    };

    priority_queue<State*, vector<State*>, StatePtrEquals> pq;
    unordered_map<pair<int, int>, pair<int, int>, PairHash, PairEquals> parents;
    vector<pair<int, int>> path;

    pq.push(new ExplorationState(xstart, ystart, 0, make_pair(-1, -1)));
    
    while (!pq.empty()) {
        auto curr = pq.top();
        pq.pop();
        
        // check if not found already
        if (parents.find(curr->get_position()) != parents.end()) {
            continue;
        }
        parents[curr->get_position()] = curr->get_parent();
        
        // found the goal? get out of here
        if (curr->is_goal(map, xmax, ymax)) {
            auto curr_point = curr->get_position();
            while (curr_point.first != -1 && curr_point.second != -1) {
                path.push_back(curr_point);
                curr_point = parents[curr_point];
            }
            delete curr;
            break;
        }
        
        // otherwise, search
        auto explore = curr->explore(map, xmax, ymax);
        for (auto it = explore.begin(); it != explore.end(); ++it) {
            if (parents.find((*it)->get_position()) == parents.end()) {
                pq.push(*it);
            } else {
                delete *it;
            }
        }
        delete curr;
    }

    while (!pq.empty()) {
        auto curr = pq.top();
        pq.pop();
        delete curr;
    }

    reverse(path.begin(), path.end());

    return path;
}

int main(void) {
    vector<vector<int>> map;

    map.push_back(vector<int>{12,18,67});
    map.push_back(vector<int>{5,104,42});
    map.push_back(vector<int>{0,10,1});

    auto path = search(map, 3, 3, 0, 0);
    
    for (const auto &coord : path) {
        cout << "(" << coord.first << "," << coord.second << ")" << endl;
    }
    
    return 0;
}

