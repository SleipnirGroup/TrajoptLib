#include <trajectory/Trajectory.h>

namespace helixtrajectory {

    Trajectory::Trajectory(const std::vector<TrajectorySample>& samples)
            : samples(samples) {
    }

    std::ostream& operator<<(std::ostream& stream, const Trajectory& trajectory) {
        
        return stream;
    }
}