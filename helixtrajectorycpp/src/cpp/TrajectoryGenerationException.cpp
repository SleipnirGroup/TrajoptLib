// Copyright (c) TrajoptLib contributors

#include "TrajectoryGenerationException.h"

#include <stdexcept>
#include <string>

namespace helixtrajectory {

TrajectoryGenerationException::TrajectoryGenerationException(
    const std::string& message)
    : logic_error(message) {}

TrajectoryGenerationException::TrajectoryGenerationException(
    const char* message)
    : logic_error(message) {}
}  // namespace helixtrajectory
