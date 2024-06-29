// Copyright (c) TrajoptLib contributors

#pragma once

#include <cstddef>
#include <memory>

#include <rust/cxx.h>

#include "trajopt/path/SwervePathBuilder.hpp"

namespace trajoptlibrust {

struct HolonomicTrajectory;
struct Pose2d;
struct SwerveDrivetrain;

class SwervePathBuilderImpl {
 public:
  SwervePathBuilderImpl() = default;

  void set_drivetrain(const SwerveDrivetrain& drivetrain);
  void set_bumpers(double length, double width);
  void set_control_interval_counts(const rust::Vec<size_t> counts);

  void pose_wpt(size_t index, double x, double y, double heading);
  void translation_wpt(size_t index, double x, double y, double heading_guess);
  void empty_wpt(size_t index, double x_guess, double y_guess,
                 double heading_guess);

  void sgmt_initial_guess_points(size_t from_index,
                                 const rust::Vec<Pose2d>& guess_points);

  void wpt_linear_velocity_direction(size_t index, double angle);
  void wpt_linear_velocity_max_magnitude(size_t index, double magnitude);
  void wpt_angular_velocity_max_magnitude(size_t index,
                                          double angular_velocity);
  void wpt_linear_acceleration_max_magnitude(size_t index, double magnitude);
  void wpt_point_at(size_t index, double field_point_x, double field_point_y,
                    double heading_tolerance);

  void sgmt_linear_velocity_direction(size_t from_index, size_t to_index,
                                      double angle);
  void sgmt_linear_velocity_max_magnitude(size_t from_index, size_t to_index,
                                          double magnitude);
  void sgmt_angular_velocity_max_magnitude(size_t from_index, size_t to_index,
                                           double angular_velocity);
  void sgmt_linear_acceleration_max_magnitude(size_t from_index,
                                              size_t to_index,
                                              double magnitude);
  void sgmt_point_at(size_t from_index, size_t to_index, double field_point_x,
                     double field_point_y, double heading_tolerance);

  void sgmt_circle_obstacle(size_t from_index, size_t to_index, double x,
                            double y, double radius);
  void sgmt_polygon_obstacle(size_t from_index, size_t to_index,
                             rust::Vec<double> x, rust::Vec<double> y,
                             double radius);

  // TODO: Return std::expected<HolonomicTrajectory, std::string> instead of
  // throwing exception, once cxx supports it
  HolonomicTrajectory generate(bool diagnostics = false,
                               int64_t handle = 0) const;

  void add_progress_callback(
      rust::Fn<void(HolonomicTrajectory, int64_t)> callback);

  HolonomicTrajectory calculate_linear_initial_guess() const;
  HolonomicTrajectory calculate_spline_initial_guess() const;
  rust::Vec<rust::usize> calculate_control_interval_counts() const;

  void cancel_all();

 private:
  trajopt::SwervePathBuilder path;
};

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl();

}  // namespace trajoptlibrust
