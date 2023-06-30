// Copyright (c) TrajoptLib contributors

#pragma once

#include <stdexcept>
#include <string>

#include "SymbolExports.h"

namespace trajopt {

/**
 * @brief This exception is thrown when an invalid path is used with a
 * trajectory generator.
 */
class TRAJOPT_DLLEXPORT InvalidPathException : public std::logic_error {
 public:
  /**
   * @brief Construct a new InvalidPathException object with a string message
   *
   * @param message the message
   */
  explicit InvalidPathException(const std::string& message);

  /**
   * @brief Construct a new InvalidPathException object with a string message
   *
   * @param message the message
   */
  explicit InvalidPathException(const char* message);
};
}  // namespace trajopt
