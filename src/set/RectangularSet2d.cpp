// Copyright (c) TrajoptLib contributors

#include "trajopt/set/RectangularSet2d.h"

#include <cmath>

#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

RectangularSet2d RectangularSet2d::PolarExactSet2d(double r, double theta) {
  return RectangularSet2d{r * std::cos(theta), r * std::sin(theta)};
}

RectangularSet2d RectangularSet2d::R2() {
  return RectangularSet2d{IntervalSet1d::R1(), IntervalSet1d::R1()};
}

bool RectangularSet2d::IsValid() const noexcept {
  return xBound.IsValid() && yBound.IsValid();
}

}  // namespace trajopt
