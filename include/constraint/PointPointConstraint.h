#pragma once

#include "set/IntervalSet1d.h"

namespace trajopt {

struct PointPointConstraint {
  double robotPointX;
  double robotPointY;
  double fieldPointX;
  double fieldPointY;
  IntervalSet1d distance;
};
}