// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/MathExtras.h"
#include "trajopt/SymbolExports.h"

#include "frc/geometry/Rotation2d.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/math.h"

namespace frc {
/**
 * Represents the position of one swerve module.
 */
struct TRAJOPT_DLLEXPORT SwerveModulePosition {
  /**
   * Distance the wheel of a module has traveled
   */
  units::meter_t distance = 0_m;

  /**
   * Angle of the module.
   */
  Rotation2d angle;

  /**
   * Checks equality between this SwerveModulePosition and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const SwerveModulePosition& other) const;

  SwerveModulePosition Interpolate(const SwerveModulePosition& endValue,
                                   double t) const {
    return {wpi::Lerp(distance, endValue.distance, t),
            wpi::Lerp(angle, endValue.angle, t)};
  }
};
}  // namespace frc
