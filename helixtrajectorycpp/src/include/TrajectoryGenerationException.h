#pragma once

#include <stdexcept>
#include <string>

namespace helixtrajectory {

    class TrajectoryGenerationException : public std::logic_error {
    public:
        explicit TrajectoryGenerationException(const std::string& message);
        explicit TrajectoryGenerationException(const char* message);
    };
}