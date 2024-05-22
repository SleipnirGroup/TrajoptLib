// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"

#include <vector>

namespace trajopt {

/**
 * Holonomic trajectory sample.
 */
class TRAJOPT_DLLEXPORT HolonomicTrajectorySample {
 public:
  /// The timestamp.
  double timestamp = 0.0;

  /// The x coordinate.
  double x = 0.0;

  /// The y coordinate.
  double y = 0.0;

  /// The heading.
  double heading = 0.0;

  /// The velocity's x component.
  double velocityX = 0.0;

  /// The velocity's y component.
  double velocityY = 0.0;

  /// The angular velocity.
  double angularVelocity = 0.0;

  /// The force on each module in the X direction.
  std::vector<double> moduleForcesX{};

  /// The force on each module in the Y direction.
  std::vector<double> moduleForcesY{};

  constexpr HolonomicTrajectorySample() = default;

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
  constexpr HolonomicTrajectorySample(double timestamp, double x, double y,
                                      double heading, double velocityX,
                                      double velocityY, double angularVelocity)
      : timestamp{timestamp},
        x{x},
        y{y},
        heading{heading},
        velocityX{velocityX},
        velocityY{velocityY},
        angularVelocity{angularVelocity}
  {}

  constexpr HolonomicTrajectorySample(double timestamp, double x, double y,
                                      double heading, double velocityX,
                                      double velocityY, double angularVelocity, const std::vector<double>& moduleForcesX, const std::vector<double>& moduleForcesY)
      : timestamp{timestamp},
        x{x},
        y{y},
        heading{heading},
        velocityX{velocityX},
        velocityY{velocityY},
        angularVelocity{angularVelocity},
  moduleForcesX(moduleForcesX), moduleForcesY(moduleForcesY)
  {}

};

}  // namespace trajopt
