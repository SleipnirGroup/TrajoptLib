// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"

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
};

}  // namespace trajopt
