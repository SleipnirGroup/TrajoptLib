#pragma once

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    class InfeasibleTrajectoryException : public std::logic_error {
    public:
        explicit InfeasibleTrajectoryException(const std::string& message);
        explicit InfeasibleTrajectoryException(const char* message);
    };
}