// Copyright (c) TrajoptLib contributors

#pragma once

#include <stdexcept>
#include <string_view>

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * @brief This exception is thrown when a trajectory generator is unable
 * to generate a trajectory.
 */
class TRAJOPT_DLLEXPORT TrajectoryGenerationException
    : public std::logic_error {
 public:
  /**
   * @brief Construct a new Trajectory Generation Exception object with a string
   * message.
   *
   * @param message the string message
   */
  explicit TrajectoryGenerationException(std::string_view message);

  /**
   * @brief Construct a new Trajectory Generation Exception object with a string
   * message.
   *
   * @param message the string message
   */
  explicit TrajectoryGenerationException(const char* message);
};
}  // namespace trajopt
