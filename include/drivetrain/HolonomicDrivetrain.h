// Copyright (c) TrajoptLib contributors

#pragma once

#include "SymbolExports.h"
#include "drivetrain/Drivetrain.h"

namespace trajopt {

/**
 * @brief This class represents a type of drivetrain that is holonomic.
 * Holonomic drivetrains have complete (or approximate) control over the three
 * degrees of freedom: position and rotation. For example, mecanum drivetrains
 * and swerve drivetrains can both manipulate their motors to have any overall
 * velocity vector while having any heading. Holonomic Drivetrain introduces no
 * new fields, but this inheritance structure is still used since it is logical
 * and maintains consistency with the rest of the api.
 */
class TRAJOPT_DLLEXPORT HolonomicDrivetrain : public Drivetrain {
 protected:
  /**
   * Construct a new HolonomicDrivetrain with the robot's mass, moment of
   * inertia, and bumpers.
   *
   * @param mass The mass of the entire robot.
   * @param momentOfInertia The moment of inertia of the robot about the center
   *   of rotation.
   */
  HolonomicDrivetrain(double mass, double momentOfInertia);
};

}  // namespace trajopt
