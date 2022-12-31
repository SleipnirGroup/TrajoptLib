#include "IncompatibleTrajectoryException.h"

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    IncompatibleTrajectoryException::IncompatibleTrajectoryException(const std::string& message)
            : logic_error(message) {
    }

    IncompatibleTrajectoryException::IncompatibleTrajectoryException(const char* message)
            : logic_error(message) {
    }
}