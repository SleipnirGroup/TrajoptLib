#pragma once

#include "SymbolExports.h"

namespace trajopt {

struct TRAJOPT_DLLEXPORT DifferentialDriverail {
  double wheelRadius;
  double wheelMaxAngularVelocity;
  double wheelMaxTorque;
};

struct TRAJOPT_DLLEXPORT DifferentialDrivetrain {
  double mass;
  double moi;
  double trackwidth;

  DifferentialDriverail left;
  DifferentialDriverail right;
};
}