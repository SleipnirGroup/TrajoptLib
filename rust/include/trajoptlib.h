#pragma once

#include <cstddef>
#include <memory>

#include <trajopt/path/SwervePathBuilder.h>

namespace trajoptlibrust {

struct SwerveDrivetrain;
struct HolonomicTrajectory;

class SwervePathBuilderImpl {
 public:
  void set_drivetrain(const SwerveDrivetrain& drivetrain);

  void pose_wpt(size_t idx, double x, double y, double heading);

  void wpt_zero_velocity(size_t idx);
  void wpt_zero_angular_velocity(size_t idx);

  HolonomicTrajectory generate() const;

  SwervePathBuilderImpl() = default;

 private:
  trajopt::SwervePathBuilder path;
};

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl();

}