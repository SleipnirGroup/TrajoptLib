// Copyright (c) TrajoptLib contributors

#pragma once

#include "SymbolExports.h"

namespace trajopt {

/**
 * @brief A point of an obstacle, usually representing a vertex of a polygon.
 */
struct TRAJOPT_DLLEXPORT ObstaclePoint {
  /**
   * @brief the x-coordinate of obstacle point
   */
  double x;
  /**
   * @brief the y-coordinate of obstacle point
   */
  double y;

  /**
   * @brief Construct a new Obstacle Point object with its position
   *
   * @param x x-coordinate
   * @param y y-coordinate
   */
  ObstaclePoint(double x, double y);
};

}  // namespace trajopt
