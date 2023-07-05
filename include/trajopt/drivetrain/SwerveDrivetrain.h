// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/drivetrain/SwerveModule.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/trajectory/HolonomicTrajectory.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * @brief This class represents a swerve drivetrain robot. It includes the
 * physical properties necessary to accurately model the dynamics of the system.
 * An arbitrary number of swerve modules can be specified, but typically it will
 * be four. The order the swerve modules are listed does not matter.
 */
struct TRAJOPT_DLLEXPORT SwerveDrivetrain {
  /// the mass of the robot
  double mass;
  /// the moment of inertial of the robot about the origin
  double moi;
  /// The list of swerve modules that make the robot move, usually one in each
  /// corner.
  std::vector<SwerveModule> modules;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(SwerveDrivetrain, mass, moi, modules)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::SwerveDrivetrain)
