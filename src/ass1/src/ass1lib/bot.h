#ifndef _ASS1_BOT_H
#define _ASS1_BOT_H

#include <utility>

using namespace std;

class Bot {
public:
    Bot() : x(x), y(y) {}

    pair<double, double> get_position() {
        return make_pair(x, y);
    }

    void set_position(double x, double y) {
        this->x = x;
        this->y = y;
    }
private:
    double x;
    double y;
};

#endif 
