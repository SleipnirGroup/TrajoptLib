#pragma once

#include "set/IntervalSet1d.h"

namespace trajopt {

struct LinePointConstraint {
  double robotLineStartX;
  double robotLineStartY;
  double robotLineEndX;
  double robotLineEndY;
  double fieldPointX;
  double fieldPointY;
  IntervalSet1d distance;
};
}
