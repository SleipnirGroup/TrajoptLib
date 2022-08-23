#include "InfeasibleTrajectoryException.h"

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    InfeasibleTrajectoryException::InfeasibleTrajectoryException(const std::string& message)
            : logic_error(message) {
    }

    InfeasibleTrajectoryException::InfeasibleTrajectoryException(const char* message)
            : logic_error(message) {
    }
}