// Copyright (c) TrajoptLib contributors

#include "trajopt/set/ManifoldIntervalSet2d.h"

#include <numbers>

namespace trajopt {

bool ManifoldIntervalSet2d::IsValid() const noexcept {
  return tolerance <= 2 * std::numbers::pi;
}

}  // namespace trajopt
