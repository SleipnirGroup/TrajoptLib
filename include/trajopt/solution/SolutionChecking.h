// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

#include "trajopt/SymbolExports.h"

/**
 * Solution error.
 */
struct TRAJOPT_DLLEXPORT SolutionError {
  /// The error message.
  std::string errorMessage;
};

/**
 * Solution tolerances.
 */
struct TRAJOPT_DLLEXPORT SolutionTolerances {
  /// The error margin.
  double errorMargin;
};
