// Copyright (c) TrajoptLib contributors

#include "obstacle/ObstaclePoint.h"

#include <iostream>

namespace trajopt {

ObstaclePoint::ObstaclePoint(double x, double y) : x(x), y(y) {}

std::ostream& operator<<(std::ostream& stream, const ObstaclePoint& point) {
  return stream << "{\"x\": " << point.x << ", \"y\": " << point.y << "}";
}
}  // namespace trajopt
