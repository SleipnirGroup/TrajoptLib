#include "trajectory/TrajectorySample.h"

namespace helixtrajectory {

    TrajectorySample::TrajectorySample(double intervalDuration, double x, double y, double heading)
            : intervalDuration(intervalDuration), x(x), y(y), heading(heading) {
    }

    std::ostream& operator<<(std::ostream& stream, const TrajectorySample& sample) {
        return stream << "{\"x\": " << sample.x
                << ", \"y\": " << sample.y
                << ", \"heading\": " << sample.heading
                << "}";
    }
}