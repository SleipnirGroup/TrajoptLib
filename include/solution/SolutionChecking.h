// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

#include "SymbolExports.h"

struct TRAJOPT_DLLEXPORT SolutionError {
  std::string errorMessage;
};

struct TRAJOPT_DLLEXPORT SolutionTolerances {
  double errorMargin;
};
