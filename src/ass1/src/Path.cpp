#include <iostream>
#include <queue>
#include <vector>

#define MAP_ROWS 3
#define MAP_COLS 3

class Path {
private:
    Path* parent;
    int x;
    int y;
    int costToCurr = 0;
    int priority;
public:
    Path(Path* parent, int x, int y) :
        parent{parent}, x{x}, y{y} {
            if (parent == NULL) {
                costToCurr = 0;
            } else {
                costToCurr = parent->costToCurr + 1;
            }
            priority = costToCurr + estimate();
    }

    int getX() const {
        return x;
    }

    int getY() const {
        return y;
    }

    int getCostToCurr() const {
        return costToCurr;
    }

    int getPriority() const {
        return priority;
    }

    void updatePriority(int p) {
        priority = p;
    }

    Path* getParent() {
        return parent;
    }

    friend bool operator<(const Path& a, const Path& b) {
        return a.getPriority() < b.getPriority();
    }

    int estimate() {
        return 0;
    }

    std::vector<std::pair<int, int>> getNeighbours() {
        std::vector<std::pair<int, int>> neighbours;

        if ((x - 1) >= 0) {
            std::pair<int, int> top(x-1, y);
            neighbours.push_back(top);
        }

        if ((x + 1) < MAP_COLS) {
            std::pair<int, int> bottom(x+1, y);
            neighbours.push_back(bottom);
        }

        if ((y - 1) >= 0) {
            std::pair<int, int> left(x, y-1);
            neighbours.push_back(left);
        }

        if ((y + 1) < MAP_ROWS) {
            std::pair<int, int> right(x, y+1);
            neighbours.push_back(right);
        }
        return neighbours;
    }
};

// Returns a vector of [x, y] coordinate pairs
// vector[0] = starting point
// vector[lastIndex] = goal point
std::vector<std::pair<int, int>> findPath(int startX, int startY, int map[MAP_ROWS][MAP_COLS], int goalValue) {
    
    std::vector<std::pair<int, int>> v;
    std::priority_queue<Path*> pq;
    
    // Add current starting cell    
    Path* p = new Path{NULL, startX, startY};
    pq.push(p);

    while (!pq.empty()) {
        Path* temp = pq.top();
        pq.pop();

        if (map[temp->getX()][temp->getY()] == goalValue) {
            v.insert(v.begin(), std::pair<int, int>(temp->getX(), temp->getY()));
            
            // Generate path
            Path* par = temp->getParent();
            while (par != NULL) {
                v.insert(v.begin(), std::pair<int, int>(par->getX(), par->getY()));
                par = par->getParent();
            }

            // TODO: Garbage collection?? 
            break;
        }

        for (auto n : temp->getNeighbours()) {
            // If it is the cell we came from, ignore it
            if (temp->getParent() != NULL && n.first == temp->getParent()->getX()
                                          && n.second == temp->getParent()->getY()) {
                    continue;
            }
            Path *neighbourNode = new Path{temp, n.first, n.second};
            int priority = neighbourNode->getCostToCurr() + map[neighbourNode->getX()][neighbourNode->getY()];
            neighbourNode->updatePriority(priority);
            pq.push(neighbourNode);
        }
    }

    return v;
}

int main (void) {
    
    int map[MAP_ROWS][MAP_COLS] = {{0,    0,  2},
                                   {100,  0,  0},
                                   {100,  5, -1}};

    std::vector<std::pair<int, int>> p = findPath(0, 0, map, -1);
    std::cout << "shortest path is " << std::endl;
    for (auto a : p) {
        std::cout << "[" << a.first << ", " << a.second << "]";
    }
    std::cout << std::endl;
    return 0;
}