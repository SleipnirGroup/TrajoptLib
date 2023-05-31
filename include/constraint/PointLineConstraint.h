// Copyright (c) TrajoptLib contributors

#pragma once

#include "set/IntervalSet1d.h"

namespace trajopt {

struct PointLineConstraint {
  double robotPointX;
  double robotPointY;
  double fieldLineStartX;
  double fieldLineStartY;
  double fieldLineEndX;
  double fieldLineEndY;
  IntervalSet1d distance;
};
}  // namespace trajopt
