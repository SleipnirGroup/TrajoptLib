// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>
#include <string>

struct SolutionError {
  std::string errorMessage;
};

struct SolutionTolerances {
  double errorMargin;
};
