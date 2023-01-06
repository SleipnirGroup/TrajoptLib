// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>
#include <vector>

#include <fmt/core.h>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <fmt/std.h>

#include "drivetrain/HolonomicDrivetrain.h"
#include "drivetrain/SwerveModule.h"
#include "obstacle/Obstacle.h"
#include "trajectory/HolonomicTrajectory.h"

namespace helixtrajectory {

/**
 * @brief This class represents a swerve drivetrain robot. It includes the
 * physical properties necessary to accurately model the dynamics of the system.
 * An arbitrary number of swerve modules can be specified, but typically it will
 * be four. The order the swerve modules are listed does not matter.
 *
 * @author Justin Babilino
 */
class SwerveDrivetrain : public HolonomicDrivetrain {
 public:
  /**
   * @brief the list of swerve modules that make the robot move, usually one in
   * each corner
   */
  std::vector<SwerveModule> modules;

  /**
   * @brief Construct a new SwerveDrivetrain with the robot's mass, moment of
   * inertia, swerve modules, and bumpers.
   *
   * @param mass the mass of the entire robot
   * @param momentOfInertia the moment of inertia of the robot about the center
   * of rotation, which
   * @param modules the list of modules the make up this swerve drivetrain
   * @param bumpers the bumpers of the robot represented as an obstacle
   */
  SwerveDrivetrain(double mass, double momentOfInertia,
                   const std::vector<SwerveModule>& modules);

  // void CheckState(const HolonomicState& state) const;
  // void CheckTrajectory(const HolonomicTrajectory& trajectory) const;

  /**
   * @brief Append a string representation of a swerve drivetrain to an output
   * stream. A string representation of a swerve drivetrain is a json object
   * with a "mass" numerical field, a "moment_of_inertia" numerical field, a
   * "bumpers" object field, and a "modules" array field.
   *
   * @param stream the stream to append the string representation to
   * @param swerveDrivetrain the swerve drivetrain
   * @return a reference to the given stream
   */
  friend std::ostream& operator<<(std::ostream& stream,
                                  const SwerveDrivetrain& swerveDrivetrain);
};
}  // namespace helixtrajectory

template <>
struct fmt::formatter<helixtrajectory::SwerveDrivetrain> {
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const helixtrajectory::SwerveDrivetrain& swerveDrivetrain,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(),
                          "swerve drivetrain:\n"
                          "  mass = {},\n"
                          "  moi = {},\n"
                          "  modules = (no impl yet)",
                          swerveDrivetrain.mass,
                          swerveDrivetrain.momentOfInertia);
  }
};
