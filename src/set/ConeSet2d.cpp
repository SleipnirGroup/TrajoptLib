// Copyright (c) TrajoptLib contributors

#include "trajopt/set/ConeSet2d.h"

#include <numbers>

namespace trajopt {

bool ConeSet2d::IsValid() const noexcept {
  return thetaBound.Range() > 0.0 && thetaBound.Range() <= std::numbers::pi;
}

}  // namespace trajopt
