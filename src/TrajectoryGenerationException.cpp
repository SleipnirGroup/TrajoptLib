// Copyright (c) TrajoptLib contributors

#include "trajopt/TrajectoryGenerationException.h"

#include <string>

namespace trajopt {

TrajectoryGenerationException::TrajectoryGenerationException(
    std::string_view message)
    : std::logic_error{std::string{message}} {}

TrajectoryGenerationException::TrajectoryGenerationException(
    const char* message)
    : std::logic_error{message} {}
}  // namespace trajopt
