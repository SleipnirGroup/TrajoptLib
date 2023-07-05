// Copyright (c) TrajoptLib contributors

#pragma once

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * @brief An initial guess of a possible state the robot may be in during the
 * trajectory.
 */
struct TRAJOPT_DLLEXPORT InitialGuessPoint {
  /// The initial guess of the x-coordinate of the robot.
  double x;

  /// The initial guess of the y-coordinate of the robot.
  double y;

  /// The initial guess of the heading of the robot.
  double heading;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    InitialGuessPoint,
    x,
    y,
    heading)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::InitialGuessPoint)
