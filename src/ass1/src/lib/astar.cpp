#include <iostream>
#include <queue>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "astar.h"

using namespace std;
        
vector<pair<int, int>> search(const Maze &maze, State *initial_state) {
    // need these for custom comparators
    class PairHash{
    public:
        size_t operator()(const pair<int, int> &k) const {
            return k.first * 100000 + k.second;
        }
    };

    struct PairEquals : binary_function<const pair<int,int>&, const pair<int,int>&, bool> {
        result_type operator()( first_argument_type lhs, second_argument_type rhs ) const {
            return (lhs.first == rhs.first) && (lhs.second == rhs.second);
        }
    };

    struct StatePtrPQEquals {
        bool operator()(const State* lhs, const State* rhs) {
            return !(*lhs < *rhs);
        }
    };

    priority_queue<State*, vector<State*>, StatePtrPQEquals> pq;
    unordered_map<pair<int, int>, pair<int, int>, PairHash, PairEquals> parents;
    vector<pair<int, int>> path;
    
    pq.push(initial_state);
    
    while (!pq.empty()) {
        auto curr = pq.top();
        pq.pop();
        
        // check if not found already
        if (parents.find(curr->get_position()) != parents.end()) {
            delete curr;
            continue;
        }
        parents[curr->get_position()] = curr->get_parent();
        
        // found the goal? get out of here
        if (curr->is_goal(maze)) {
            auto curr_point = curr->get_position();
            while (curr_point.first != -1 && curr_point.second != -1) {
                path.push_back(curr_point);
                curr_point = parents[curr_point];
            }
            delete curr;
            break;
        }
        
        // otherwise, search
        auto explore = curr->explore(maze,
                [&parents](pair<int, int> point) { return parents.find(point) == parents.end(); });
        for (auto it = explore.begin(); it != explore.end(); ++it) {
            pq.push(*it);
        }
        delete curr;
    }

    // prevent memory leaks
    while (!pq.empty()) {
        auto curr = pq.top();
        pq.pop();
        delete curr;
    }

    // path is the reverse of where we are
    reverse(path.begin(), path.end());

    return path;
}

