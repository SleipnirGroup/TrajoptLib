// Copyright (c) TrajoptLib contributors

#pragma once

#include <trajopt/path/SwervePathBuilder.h>

#include <cstddef>
#include <memory>

namespace trajoptlibrust {

struct SwerveDrivetrain;
struct HolonomicTrajectory;

class SwervePathBuilderImpl {
 public:
  void set_drivetrain(const SwerveDrivetrain& drivetrain);

  void translation_wpt(size_t idx, double x, double y, double heading_guess);
  void pose_wpt(size_t idx, double x, double y, double heading);

  void wpt_zero_velocity(size_t idx);
  void wpt_zero_angular_velocity(size_t idx);
  void wpt_velocity_direction(size_t idx, double angle);
  void wpt_velocity_magnitude(size_t idx, double v);
  void wpt_velocity_polar(size_t idx, double vr, double vtheta);
  void wpt_angular_velocity(size_t idx, double angular_velocity);

  HolonomicTrajectory generate() const;

  SwervePathBuilderImpl() = default;

 private:
  trajopt::SwervePathBuilder path;
};

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl();

}  // namespace trajoptlibrust
