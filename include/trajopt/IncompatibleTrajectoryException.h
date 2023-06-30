// Copyright (c) TrajoptLib contributors

#pragma once

#include <stdexcept>
#include <string>

#include "SymbolExports.h"

namespace trajopt {

/**
 * Incompatible trajectory exception.
 */
class TRAJOPT_DLLEXPORT IncompatibleTrajectoryException
    : public std::logic_error {
 public:
  /**
   * Construct a new IncompatibleTrajectoryException object with a string
   * message
   *
   * @param message the message
   */
  explicit IncompatibleTrajectoryException(const std::string& message);

  /**
   * Construct a new IncompatibleTrajectoryException object with a string
   * message
   *
   * @param message the message
   */
  explicit IncompatibleTrajectoryException(const char* message);
};
}  // namespace trajopt
