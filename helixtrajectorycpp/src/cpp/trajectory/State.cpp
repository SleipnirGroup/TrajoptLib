#include "trajectory/State.h"

namespace helixtrajectory {

State::State(double x, double y, double heading)
        : x(x), y(y), heading(heading) {
}

std::ostream& operator<<(std::ostream& stream, const State& state) {
    return stream << "{\"x\": " << state.x
            << ", \"y\": " << state.y
            << ", \"heading\": " << state.heading
            << "}";
}
}