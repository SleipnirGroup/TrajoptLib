#include "trajectory/State.h"

#include <iostream>
#include <fmt/format.h>

namespace helixtrajectory {

State::State(double x, double y, double heading)
        : x(x), y(y), heading(heading) {
}

std::ostream& operator<<(std::ostream& stream, const State& state) {
    return stream << fmt::format("{{\"x\": {}, \"y\": {}, \"heading\": {}}}",
            state.x, state.y, state.heading);
}
}