// Copyright (c) TrajoptLib contributors

#include "trajoptlibrust.hpp"

#include <stdint.h>

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

#include "trajopt/OptimalTrajectoryGenerator.hpp"
#include "trajopt/drivetrain/SwerveDrivetrain.hpp"
#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/trajectory/HolonomicTrajectory.hpp"
#include "trajopt/trajectory/HolonomicTrajectorySample.hpp"
#include "trajoptlib/src/lib.rs.h"

namespace trajoptlibrust {

template <typename FromType, typename ToType, typename FromVecType,
          typename ToVecType, ToType (*Converter)(const FromType&)>
ToVecType _convert_generic_vec(const FromVecType& fromVec) {
  ToVecType toVec;
  toVec.reserve(fromVec.size());
  for (const FromType& item : fromVec) {
    toVec.emplace_back(Converter(item));
  }
  return toVec;
}

template <typename RustType, typename CppType,
          CppType (*Converter)(const RustType&)>
std::vector<CppType> _rust_vec_to_cpp_vector(
    const rust::Vec<RustType>& rustVec) {
  return _convert_generic_vec<RustType, CppType, rust::Vec<RustType>,
                              std::vector<CppType>, Converter>(rustVec);
}

template <typename CppType, typename RustType,
          RustType (*Converter)(const CppType&)>
rust::Vec<RustType> _cpp_vector_to_rust_vec(
    const std::vector<CppType>& cppVec) {
  return _convert_generic_vec<CppType, RustType, std::vector<CppType>,
                              rust::Vec<RustType>, Converter>(cppVec);
}

trajopt::SwerveModule _convert_swerve_module(const SwerveModule& swerveModule) {
  return trajopt::SwerveModule{
      trajopt::Translation2d{swerveModule.x, swerveModule.y},
      swerveModule.wheel_radius, swerveModule.wheel_max_angular_velocity,
      swerveModule.wheel_max_torque};
}

trajopt::SwerveDrivetrain _convert_swerve_drivetrain(
    const SwerveDrivetrain& drivetrain) {
  return trajopt::SwerveDrivetrain{
      .mass = drivetrain.mass,
      .moi = drivetrain.moi,
      .modules =
          _rust_vec_to_cpp_vector<SwerveModule, trajopt::SwerveModule,
                                  &_convert_swerve_module>(drivetrain.modules)};
}

trajopt::Pose2d _convert_initial_guess_point(
    const InitialGuessPoint& initialGuessPoint) {
  return {initialGuessPoint.x, initialGuessPoint.y, initialGuessPoint.heading};
}

void SwervePathBuilderImpl::set_drivetrain(const SwerveDrivetrain& drivetrain) {
  path.SetDrivetrain(_convert_swerve_drivetrain(drivetrain));
}

size_t _convert_count(const size_t& count) {
  return count;
}

void SwervePathBuilderImpl::set_control_interval_counts(
    const rust::Vec<size_t> counts) {
  std::vector<size_t> converted_counts =
      _rust_vec_to_cpp_vector<size_t, size_t, &_convert_count>(counts);
  path.ControlIntervalCounts(std::move(converted_counts));
}

void SwervePathBuilderImpl::set_bumpers(double length, double width) {
  path.AddBumpers(trajopt::Bumpers{.safetyDistance = 0.01,
                                   .points = {{+length / 2, +width / 2},
                                              {-length / 2, +width / 2},
                                              {-length / 2, -width / 2},
                                              {+length / 2, -width / 2}}});
}

void SwervePathBuilderImpl::pose_wpt(size_t idx, double x, double y,
                                     double heading) {
  path.PoseWpt(idx, x, y, heading);
}

void SwervePathBuilderImpl::translation_wpt(size_t idx, double x, double y,
                                            double heading_guess) {
  path.TranslationWpt(idx, x, y, heading_guess);
}

void SwervePathBuilderImpl::empty_wpt(size_t idx, double x_guess,
                                      double y_guess, double heading_guess) {
  path.WptInitialGuessPoint(idx, {x_guess, y_guess, heading_guess});
}

void SwervePathBuilderImpl::sgmt_initial_guess_points(
    size_t from_idx, const rust::Vec<InitialGuessPoint>& guess_points) {
  std::vector<trajopt::Pose2d> convertedGuessPoints =
      _rust_vec_to_cpp_vector<InitialGuessPoint, trajopt::Pose2d,
                              &_convert_initial_guess_point>(guess_points);
  path.SgmtInitialGuessPoints(from_idx, convertedGuessPoints);
}

void SwervePathBuilderImpl::wpt_linear_velocity_direction(size_t idx,
                                                          double angle) {
  path.WptVelocityDirection(idx, angle);
}

void SwervePathBuilderImpl::wpt_linear_velocity_max_magnitude(
    size_t idx, double magnitude) {
  path.WptVelocityMagnitude(idx, magnitude);
}

void SwervePathBuilderImpl::wpt_linear_velocity_polar(size_t idx,
                                                      double magnitude,
                                                      double angle) {
  path.WptVelocityPolar(idx, magnitude, angle);
}

void SwervePathBuilderImpl::wpt_angular_velocity(size_t idx,
                                                 double angular_velocity) {
  // this probably ought to be added to SwervePathBuilder in the C++ API
  path.WptAngularVelocity(idx, angular_velocity);
}

void SwervePathBuilderImpl::wpt_angular_velocity_max_magnitude(
    size_t idx, double angular_velocity) {
  path.WptAngularVelocityMaxMagnitude(idx, angular_velocity);
}

void SwervePathBuilderImpl::wpt_x(size_t idx, double x) {
  path.WptConstraint(idx,
                     trajopt::TranslationConstraint{trajopt::RectangularSet2d{
                         .xBound = x, .yBound = trajopt::IntervalSet1d::R1()}});
}

void SwervePathBuilderImpl::wpt_y(size_t idx, double y) {
  path.WptConstraint(idx,
                     trajopt::TranslationConstraint{trajopt::RectangularSet2d{
                         .xBound = trajopt::IntervalSet1d::R1(), .yBound = y}});
}

void SwervePathBuilderImpl::wpt_heading(size_t idx, double heading) {
  path.WptConstraint(idx, trajopt::HeadingConstraint{heading});
}

void SwervePathBuilderImpl::wpt_point_at(size_t idx, double field_point_x,
                                         double field_point_y,
                                         double heading_tolerance) {
  path.WptConstraint(idx,
                     trajopt::PointAtConstraint{
                         trajopt::Translation2d{field_point_x, field_point_y},
                         heading_tolerance});
}

void SwervePathBuilderImpl::sgmt_linear_velocity_direction(size_t from_idx,
                                                           size_t to_idx,
                                                           double angle) {
  path.SgmtVelocityDirection(from_idx, to_idx, angle);
}

void SwervePathBuilderImpl::sgmt_linear_velocity_max_magnitude(
    size_t from_idx, size_t to_idx, double magnitude) {
  path.SgmtVelocityMagnitude(from_idx, to_idx, magnitude);
}

void SwervePathBuilderImpl::sgmt_linear_velocity_polar(size_t from_idx,
                                                       size_t to_idx,
                                                       double magnitude,
                                                       double angle) {
  path.SgmtConstraint(
      from_idx, to_idx,
      trajopt::HolonomicVelocityConstraint{
          trajopt::RectangularSet2d::PolarExactSet2d(magnitude, angle),
          trajopt::CoordinateSystem::kField});
}

void SwervePathBuilderImpl::sgmt_angular_velocity(size_t from_idx,
                                                  size_t to_idx,
                                                  double angular_velocity) {
  path.SgmtAngularVelocity(from_idx, to_idx, angular_velocity);
}

void SwervePathBuilderImpl::sgmt_angular_velocity_max_magnitude(
    size_t from_idx, size_t to_idx, double angular_velocity) {
  path.SgmtAngularVelocityMaxMagnitude(from_idx, to_idx, angular_velocity);
}

void SwervePathBuilderImpl::sgmt_x(size_t from_idx, size_t to_idx, double x) {
  path.SgmtConstraint(
      from_idx, to_idx,
      trajopt::TranslationConstraint{trajopt::RectangularSet2d{
          .xBound = x, .yBound = trajopt::IntervalSet1d::R1()}});
}

void SwervePathBuilderImpl::sgmt_y(size_t from_idx, size_t to_idx, double y) {
  path.SgmtConstraint(
      from_idx, to_idx,
      trajopt::TranslationConstraint{trajopt::RectangularSet2d{
          .xBound = trajopt::IntervalSet1d::R1(), .yBound = y}});
}

void SwervePathBuilderImpl::sgmt_heading(size_t from_idx, size_t to_idx,
                                         double heading) {
  path.SgmtConstraint(from_idx, to_idx, trajopt::HeadingConstraint{heading});
}

void SwervePathBuilderImpl::sgmt_point_at(size_t from_idx, size_t to_idx,
                                          double field_point_x,
                                          double field_point_y,
                                          double heading_tolerance) {
  path.SgmtConstraint(from_idx, to_idx,
                      trajopt::PointAtConstraint{
                          trajopt::Translation2d{field_point_x, field_point_y},
                          heading_tolerance});
}

void SwervePathBuilderImpl::sgmt_circle_obstacle(size_t from_idx, size_t to_idx,
                                                 double x, double y,
                                                 double radius) {
  auto obstacle =
      trajopt::Obstacle{.safetyDistance = radius, .points = {{x, y}}};
  path.SgmtObstacle(from_idx, to_idx, obstacle);
}

void SwervePathBuilderImpl::sgmt_polygon_obstacle(size_t from_idx,
                                                  size_t to_idx,
                                                  const rust::Vec<double> x,
                                                  const rust::Vec<double> y,
                                                  double radius) {
  std::vector<trajopt::Translation2d> points;
  if (x.size() != y.size()) {
    return;
  }
  for (size_t i = 0; i < x.size(); i++) {
    points.push_back({x.at(i), y.at(i)});
  }
  auto obstacle = trajopt::Obstacle{.safetyDistance = radius, .points = points};
  path.SgmtObstacle(from_idx, to_idx, obstacle);
}

HolonomicTrajectorySample _convert_holonomic_trajectory_sample(
    const trajopt::HolonomicTrajectorySample& sample) {
  // copy data into rust vecs
  rust::Vec<double> fx;
  std::copy(sample.moduleForcesX.begin(), sample.moduleForcesX.end(),
            std::back_inserter(fx));

  rust::Vec<double> fy;
  std::copy(sample.moduleForcesY.begin(), sample.moduleForcesY.end(),
            std::back_inserter(fy));

  return HolonomicTrajectorySample{
      .timestamp = sample.timestamp,
      .x = sample.x,
      .y = sample.y,
      .heading = sample.heading,
      .velocity_x = sample.velocityX,
      .velocity_y = sample.velocityY,
      .angular_velocity = sample.angularVelocity,
      .module_forces_x = std::move(fx),
      .module_forces_y = std::move(fy),
  };
}

HolonomicTrajectory _convert_holonomic_trajectory(
    const trajopt::HolonomicTrajectory& trajectory) {
  return HolonomicTrajectory{
      .samples = _cpp_vector_to_rust_vec<trajopt::HolonomicTrajectorySample,
                                         HolonomicTrajectorySample,
                                         &_convert_holonomic_trajectory_sample>(
          trajectory.samples)};
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
  path.AddIntermediateCallback([=](trajopt::SwerveSolution& solution,
                                   int64_t handle) {
    callback(
        _convert_holonomic_trajectory(trajopt::HolonomicTrajectory{solution}),
        handle);
  });
}

HolonomicTrajectory SwervePathBuilderImpl::generate(bool diagnostics,
                                                    int64_t handle) const {
  if (auto sol = trajopt::OptimalTrajectoryGenerator::Generate(
          path, diagnostics, handle);
      sol.has_value()) {
    return _convert_holonomic_trajectory(
        trajopt::HolonomicTrajectory{sol.value()});
  } else {
    throw std::runtime_error{sol.error()};
  }
}

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl() {
  return std::make_unique<SwervePathBuilderImpl>(SwervePathBuilderImpl());
}

void SwervePathBuilderImpl::cancel_all() {
  path.CancelAll();
}

}  // namespace trajoptlibrust
