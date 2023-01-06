// Copyright (c) TrajoptLib contributors

#include "IncompatibleTrajectoryException.h"

namespace trajopt {

IncompatibleTrajectoryException::IncompatibleTrajectoryException(
    const std::string& message)
    : std::logic_error(message) {}

IncompatibleTrajectoryException::IncompatibleTrajectoryException(
    const char* message)
    : std::logic_error(message) {}
}  // namespace trajopt
