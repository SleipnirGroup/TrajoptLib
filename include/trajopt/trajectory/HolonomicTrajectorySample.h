// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * Holonomic trajectory sample.
 */
class TRAJOPT_DLLEXPORT HolonomicTrajectorySample {
 public:
  /// The timestamp.
  double timestamp;

  /// The x coordinate.
  double x;

  /// The y coordinate.
  double y;

  /// The heading.
  double heading;

  /// The velocity's x component.
  double velocityX;

  /// The velocity's y component.
  double velocityY;

  /// The angular velocity.
  double angularVelocity;

  /**
   * Construct a HolonomicTrajectorySample.
   *
   * @param timestamp The timestamp.
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @param heading The heading.
   * @param velocityX The velocity's x component.
   * @param velocityY The velocity's y component.
   * @param angularVelocity The angular velocity.
   */
  HolonomicTrajectorySample(double timestamp, double x, double y,
                            double heading, double velocityX, double velocityY,
                            double angularVelocity);
};

}  // namespace trajopt
