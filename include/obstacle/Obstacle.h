// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "obstacle/ObstaclePoint.h"

namespace trajopt {

/**
 * @brief Represents a physical obstacle that the robot must avoid by a certain
 * distance. Arbitrary polygons can be expressed with this class, and circle
 * obstacles can also be created by only using one point with a safety distance.
 * Obstacle points must be wound either clockwise or counterclockwise.
 */
class TRAJOPT_DLLEXPORT Obstacle {
 public:
  /// Minimum distance from the obstacle the robot must maintain.
  double safetyDistance;

  /// The list of points that make up this obstacle.
  std::vector<ObstaclePoint> points;

  /**
   * Construct a Obstacle.
   *
   * @param safetyDistance Minimum distance from the obstacle the robot must
   *   maintain.
   * @param points The list of points that make up this obstacle.
   */
  Obstacle(double safetyDistance, std::vector<ObstaclePoint> points);
};

}  // namespace trajopt
