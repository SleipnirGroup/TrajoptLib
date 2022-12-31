#pragma once

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    class IncompatibleTrajectoryException : public std::logic_error {
    public:

        explicit IncompatibleTrajectoryException(const std::string& message);
        explicit IncompatibleTrajectoryException(const char* message);
    };
}