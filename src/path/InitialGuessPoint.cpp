// Copyright (c) TrajoptLib contributors

#include "path/InitialGuessPoint.h"

#include <iostream>

namespace trajopt {

InitialGuessPoint::InitialGuessPoint(double x, double y, double heading)
    : x(x), y(y), heading(heading) {}

std::ostream& operator<<(std::ostream& stream,
                         const InitialGuessPoint& guessPoint) {
  return stream << "{\"x\": " << guessPoint.x << ", \"y\": " << guessPoint.y
                << ", \"heading\": " << guessPoint.heading << "}";
}
}  // namespace trajopt
