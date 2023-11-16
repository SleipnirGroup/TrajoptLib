// Copyright (c) TrajoptLib contributors

#pragma once

#include <rust/cxx.h>
#include <trajopt/path/SwervePathBuilder.h>

#include <cstddef>
#include <memory>

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
  void wpt_x(size_t idx, double x);
  void wpt_y(size_t idx, double y);
  void wpt_heading(size_t idx, double heading);

  void sgmt_linear_velocity_direction(size_t from_idx, size_t to_idx,
                                      double angle);
  void sgmt_linear_velocity_max_magnitude(size_t from_idx, size_t to_idx,
                                          double magnitude);
  void sgmt_linear_velocity_polar(size_t from_idx, size_t to_idx,
                                  double magnitude, double angle);
  void sgmt_angular_velocity(size_t from_idx, size_t to_idx,
                             double angular_velocity);
  void sgmt_x(size_t from_idx, size_t to_idx, double x);
  void sgmt_y(size_t from_idx, size_t to_idx, double y);
  void sgmt_heading(size_t from_idx, size_t to_idx, double heading);

  void sgmt_circle_obstacle(size_t from_idx, size_t to_idx, double x, double y,
                            double radius);

  HolonomicTrajectory generate() const;
  void cancel_all();

  SwervePathBuilderImpl() = default;

 private:
  trajopt::SwervePathBuilder path;
};

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl();
}  // namespace trajoptlibrust
