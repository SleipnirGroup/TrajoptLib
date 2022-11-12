#include <trajectory/Trajectory.h>

#include <iostream>

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    Trajectory::Trajectory(const std::vector<TrajectorySample>& samples)
            : samples(samples) {
    }

    std::ostream& operator<<(std::ostream& stream, const Trajectory& trajectory) {
        stream << "[";
        double ts = 0.0;
        for (auto& sample : trajectory.samples) {
            ts += sample.intervalDuration;
            stream << "{\"ts\": " << ts
                << ", \"x\": " << sample.x
                << ", \"y\": " << sample.y
                << ", \"heading\": " << sample.heading
                << ", \"vx\": " << 0.0
                << ", \"vy\": " << 0.0
                << ", \"omega\": " << 0.0
                << "},";
        }
        return stream;
    }
}