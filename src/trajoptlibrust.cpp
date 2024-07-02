// Copyright (c) TrajoptLib contributors

#include "trajoptlibrust.hpp"

#include <stdint.h>

#include <cstddef>
#include <stdexcept>
#include <vector>

#include "trajopt/SwerveTrajectoryGenerator.hpp"
#include "trajopt/constraint/AngularVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/LinearAccelerationMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/LinearVelocityDirectionConstraint.hpp"
#include "trajopt/constraint/LinearVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/PointAtConstraint.hpp"
#include "trajopt/drivetrain/SwerveModule.hpp"
#include "trajopt/trajectory/HolonomicTrajectory.hpp"
#include "trajopt/trajectory/HolonomicTrajectorySample.hpp"
#include "trajoptlib/src/lib.rs.h"

namespace trajoptlibrust {

void SwervePathBuilderImpl::set_drivetrain(const SwerveDrivetrain& drivetrain) {
  std::vector<trajopt::SwerveModule> cppModules;
  for (const auto& module : drivetrain.modules) {
    cppModules.push_back(
        trajopt::SwerveModule{{module.x, module.y},
                              module.wheel_radius,
                              module.wheel_max_angular_velocity,
                              module.wheel_max_torque});
  }

  path.SetDrivetrain(trajopt::SwerveDrivetrain{drivetrain.mass, drivetrain.moi,
                                               std::move(cppModules)});
}

void SwervePathBuilderImpl::set_control_interval_counts(
    const rust::Vec<size_t> counts) {
  std::vector<size_t> cppCounts;
  for (const auto& count : counts) {
    cppCounts.emplace_back(count);
  }

  path.ControlIntervalCounts(std::move(cppCounts));
}

void SwervePathBuilderImpl::set_bumpers(double length, double width) {
  path.AddBumpers(trajopt::Bumpers{.safetyDistance = 0.01,
                                   .points = {{+length / 2, +width / 2},
                                              {-length / 2, +width / 2},
                                              {-length / 2, -width / 2},
                                              {+length / 2, -width / 2}}});
}

void SwervePathBuilderImpl::pose_wpt(size_t index, double x, double y,
                                     double heading) {
  path.PoseWpt(index, x, y, heading);
}

void SwervePathBuilderImpl::translation_wpt(size_t index, double x, double y,
                                            double heading_guess) {
  path.TranslationWpt(index, x, y, heading_guess);
}

void SwervePathBuilderImpl::empty_wpt(size_t index, double x_guess,
                                      double y_guess, double heading_guess) {
  path.WptInitialGuessPoint(index, {x_guess, y_guess, heading_guess});
}

void SwervePathBuilderImpl::sgmt_initial_guess_points(
    size_t from_index, const rust::Vec<Pose2d>& guess_points) {
  std::vector<trajopt::Pose2d> cppGuessPoints;
  for (const auto& guess_point : guess_points) {
    cppGuessPoints.emplace_back(guess_point.x, guess_point.y,
                                guess_point.heading);
  }

  path.SgmtInitialGuessPoints(from_index, std::move(cppGuessPoints));
}

void SwervePathBuilderImpl::wpt_linear_velocity_direction(size_t index,
                                                          double angle) {
  path.WptConstraint(index, trajopt::LinearVelocityDirectionConstraint{angle});
}

void SwervePathBuilderImpl::wpt_linear_velocity_max_magnitude(
    size_t index, double magnitude) {
  path.WptConstraint(index,
                     trajopt::LinearVelocityMaxMagnitudeConstraint{magnitude});
}

void SwervePathBuilderImpl::wpt_angular_velocity_max_magnitude(
    size_t index, double angular_velocity) {
  path.WptConstraint(
      index, trajopt::AngularVelocityMaxMagnitudeConstraint{angular_velocity});
}

void SwervePathBuilderImpl::wpt_linear_acceleration_max_magnitude(
    size_t index, double magnitude) {
  path.WptConstraint(
      index, trajopt::LinearAccelerationMaxMagnitudeConstraint{magnitude});
}

void SwervePathBuilderImpl::wpt_point_at(size_t index, double field_point_x,
                                         double field_point_y,
                                         double heading_tolerance) {
  path.WptConstraint(index,
                     trajopt::PointAtConstraint{
                         trajopt::Translation2d{field_point_x, field_point_y},
                         heading_tolerance});
}

void SwervePathBuilderImpl::sgmt_linear_velocity_direction(size_t from_index,
                                                           size_t to_index,
                                                           double angle) {
  path.SgmtConstraint(from_index, to_index,
                      trajopt::LinearVelocityDirectionConstraint{angle});
}

void SwervePathBuilderImpl::sgmt_linear_velocity_max_magnitude(
    size_t from_index, size_t to_index, double magnitude) {
  path.SgmtConstraint(from_index, to_index,
                      trajopt::LinearVelocityMaxMagnitudeConstraint{magnitude});
}

void SwervePathBuilderImpl::sgmt_angular_velocity_max_magnitude(
    size_t from_index, size_t to_index, double angular_velocity) {
  path.SgmtConstraint(
      from_index, to_index,
      trajopt::AngularVelocityMaxMagnitudeConstraint{angular_velocity});
}

void SwervePathBuilderImpl::sgmt_linear_acceleration_max_magnitude(
    size_t from_index, size_t to_index, double magnitude) {
  path.SgmtConstraint(
      from_index, to_index,
      trajopt::LinearAccelerationMaxMagnitudeConstraint{magnitude});
}

void SwervePathBuilderImpl::sgmt_point_at(size_t from_index, size_t to_index,
                                          double field_point_x,
                                          double field_point_y,
                                          double heading_tolerance) {
  path.SgmtConstraint(from_index, to_index,
                      trajopt::PointAtConstraint{{field_point_x, field_point_y},
                                                 heading_tolerance});
}

void SwervePathBuilderImpl::sgmt_circle_obstacle(size_t from_index,
                                                 size_t to_index, double x,
                                                 double y, double radius) {
  path.SgmtObstacle(from_index, to_index, {radius, {{x, y}}});
}

void SwervePathBuilderImpl::sgmt_polygon_obstacle(size_t from_index,
                                                  size_t to_index,
                                                  const rust::Vec<double> x,
                                                  const rust::Vec<double> y,
                                                  double radius) {
  if (x.size() != y.size()) [[unlikely]] {
    return;
  }

  std::vector<trajopt::Translation2d> cppPoints;
  for (size_t i = 0; i < x.size(); ++i) {
    cppPoints.emplace_back(x.at(i), y.at(i));
  }

  path.SgmtObstacle(from_index, to_index,
                    trajopt::Obstacle{.safetyDistance = radius,
                                      .points = std::move(cppPoints)});
}

HolonomicTrajectory SwervePathBuilderImpl::generate(bool diagnostics,
                                                    int64_t handle) const {
  trajopt::SwerveTrajectoryGenerator generator{path, handle};
  if (auto sol = generator.Generate(diagnostics); sol.has_value()) {
    trajopt::HolonomicTrajectory cppTrajectory{sol.value()};

    rust::Vec<HolonomicTrajectorySample> rustSamples;
    for (const auto& cppSample : cppTrajectory.samples) {
      rust::Vec<double> fx;
      std::copy(cppSample.moduleForcesX.begin(), cppSample.moduleForcesX.end(),
                std::back_inserter(fx));

      rust::Vec<double> fy;
      std::copy(cppSample.moduleForcesY.begin(), cppSample.moduleForcesY.end(),
                std::back_inserter(fy));

      rustSamples.push_back(HolonomicTrajectorySample{
          cppSample.timestamp, cppSample.x, cppSample.y, cppSample.heading,
          cppSample.velocityX, cppSample.velocityY, cppSample.angularVelocity,
          std::move(fx), std::move(fy)});
    }

    return HolonomicTrajectory{std::move(rustSamples)};
  } else {
    throw std::runtime_error{sol.error()};
  }
}

/**
 * Add a callback that will be called on each iteration of the solver.
 *
 * @param callback: a `fn` (not a closure) to be executed. The callback's
 * first parameter will be a `trajoptlib::HolonomicTrajectory`, and the second
 * parameter will be an `i64` equal to the handle passed in `generate()`
 *
 * This function can be called multiple times to add multiple callbacks.
 */
void SwervePathBuilderImpl::add_progress_callback(
    rust::Fn<void(HolonomicTrajectory, int64_t)> callback) {
  path.AddIntermediateCallback(
      [=](trajopt::SwerveSolution& solution, int64_t handle) {
        trajopt::HolonomicTrajectory cppTrajectory{solution};

        rust::Vec<HolonomicTrajectorySample> rustSamples;
        for (const auto& cppSample : cppTrajectory.samples) {
          rust::Vec<double> fx;
          std::copy(cppSample.moduleForcesX.begin(),
                    cppSample.moduleForcesX.end(), std::back_inserter(fx));

          rust::Vec<double> fy;
          std::copy(cppSample.moduleForcesY.begin(),
                    cppSample.moduleForcesY.end(), std::back_inserter(fy));

          rustSamples.push_back(HolonomicTrajectorySample{
              cppSample.timestamp, cppSample.x, cppSample.y, cppSample.heading,
              cppSample.velocityX, cppSample.velocityY,
              cppSample.angularVelocity, std::move(fx), std::move(fy)});
        }

        callback(HolonomicTrajectory{rustSamples}, handle);
      });
}

void SwervePathBuilderImpl::cancel_all() {
  path.CancelAll();
}

HolonomicTrajectory _convert_sol_to_holonomic_trajectory(
    const trajopt::SwerveSolution& solution) {
  trajopt::HolonomicTrajectory cppTrajectory{solution};

  rust::Vec<HolonomicTrajectorySample> rustSamples;
  for (const auto& cppSample : cppTrajectory.samples) {
    rust::Vec<double> fx;
    std::copy(cppSample.moduleForcesX.begin(), cppSample.moduleForcesX.end(),
              std::back_inserter(fx));

    rust::Vec<double> fy;
    std::copy(cppSample.moduleForcesY.begin(), cppSample.moduleForcesY.end(),
              std::back_inserter(fy));

    rustSamples.push_back(HolonomicTrajectorySample{
        cppSample.timestamp, cppSample.x, cppSample.y, cppSample.heading,
        cppSample.velocityX, cppSample.velocityY, cppSample.angularVelocity,
        std::move(fx), std::move(fy)});
  }
  return HolonomicTrajectory{rustSamples};
}

HolonomicTrajectory SwervePathBuilderImpl::calculate_linear_initial_guess()
    const {
  return _convert_sol_to_holonomic_trajectory(path.CalculateInitialGuess());
}

HolonomicTrajectory SwervePathBuilderImpl::calculate_spline_initial_guess()
    const {
  return _convert_sol_to_holonomic_trajectory(
      path.CalculateSplineInitialGuess());
}

rust::Vec<rust::usize>
SwervePathBuilderImpl::calculate_control_interval_counts() const {
  auto cppCounts = path.CalculateControlIntervalCounts();
  rust::Vec<rust::usize> rustCounts;
  for (const auto count : cppCounts) {
    rustCounts.emplace_back(count);
  }
  return rustCounts;
}

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl() {
  return std::make_unique<SwervePathBuilderImpl>(SwervePathBuilderImpl());
}

}  // namespace trajoptlibrust
