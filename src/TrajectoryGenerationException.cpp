// Copyright (c) TrajoptLib contributors

#include "trajopt/TrajectoryGenerationException.h"

#include <stdexcept>
#include <string>

namespace trajopt {

TrajectoryGenerationException::TrajectoryGenerationException(
    const std::string& message)
    : logic_error(message) {}

TrajectoryGenerationException::TrajectoryGenerationException(
    const char* message)
    : logic_error(message) {}
}  // namespace trajopt
