// Copyright (c) TrajoptLib contributors

#pragma once

#include <cstddef>
#include <memory>

#include <rust/cxx.h>

#include "trajopt/path/SwervePathBuilder.hpp"

namespace trajoptlibrust {

struct HolonomicTrajectory;
struct InitialGuessPoint;
struct SwerveDrivetrain;

class SwervePathBuilderImpl {
 public:
  void set_drivetrain(const SwerveDrivetrain& drivetrain);
  void set_bumpers(double length, double width);
  void set_control_interval_counts(const rust::Vec<size_t> counts);

  void pose_wpt(size_t idx, double x, double y, double heading);
  void translation_wpt(size_t idx, double x, double y, double heading_guess);
  void empty_wpt(size_t idx, double x_guess, double y_guess,
                 double heading_guess);

  void sgmt_initial_guess_points(
      size_t from_idx, const rust::Vec<InitialGuessPoint>& guess_points);

  void wpt_linear_velocity_direction(size_t idx, double angle);
  void wpt_linear_velocity_max_magnitude(size_t idx, double magnitude);
  void wpt_linear_velocity_polar(size_t idx, double magnitude, double angle);
  void wpt_angular_velocity(size_t idx, double angular_velocity);
  void wpt_angular_velocity_max_magnitude(size_t idx, double angular_velocity);
  void wpt_x(size_t idx, double x);
  void wpt_y(size_t idx, double y);
  void wpt_heading(size_t idx, double heading);
  void wpt_point_at(size_t idx, double field_point_x, double field_point_y,
                    double heading_tolerance);

  void sgmt_linear_velocity_direction(size_t from_idx, size_t to_idx,
                                      double angle);
  void sgmt_linear_velocity_max_magnitude(size_t from_idx, size_t to_idx,
                                          double magnitude);
  void sgmt_linear_velocity_polar(size_t from_idx, size_t to_idx,
                                  double magnitude, double angle);
  void sgmt_angular_velocity(size_t from_idx, size_t to_idx,
                             double angular_velocity);
  void sgmt_angular_velocity_max_magnitude(size_t from_idx, size_t to_idx,
                                           double angular_velocity);
  void sgmt_x(size_t from_idx, size_t to_idx, double x);
  void sgmt_y(size_t from_idx, size_t to_idx, double y);
  void sgmt_heading(size_t from_idx, size_t to_idx, double heading);
  void sgmt_point_at(size_t from_idx, size_t to_idx, double field_point_x,
                     double field_point_y, double heading_tolerance);

  void sgmt_circle_obstacle(size_t from_idx, size_t to_idx, double x, double y,
                            double radius);
  void sgmt_polygon_obstacle(size_t from_idx, size_t to_idx,
                             rust::Vec<double> x, rust::Vec<double> y,
                             double radius);
  // TODO: Return std::expected<HolonomicTrajectory, std::string> instead of
  // throwing exception, once cxx supports it
  HolonomicTrajectory generate(bool diagnostics = false,
                               int64_t handle = 0) const;
  void add_progress_callback(
      rust::Fn<void(HolonomicTrajectory, int64_t)> callback);
  void cancel_all();

  SwervePathBuilderImpl() = default;

 private:
  trajopt::SwervePathBuilder path;
};

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl();

}  // namespace trajoptlibrust
