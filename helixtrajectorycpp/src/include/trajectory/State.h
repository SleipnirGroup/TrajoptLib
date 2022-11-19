#pragma once

#include <iostream>

namespace helixtrajectory {

class State {
public:
    double x, y, heading;

    State(double x, double y, double heading);

    friend std::ostream& operator<<(std::ostream& stream, const State& state);
};
}