#include "trajoptlib/include/trajoptlib.h"

#include <cstddef>
#include <memory>
#include <vector>

#include <trajopt/path/SwervePathBuilder.h>
#include <trajopt/drivetrain/SwerveDrivetrain.h>
#include <trajopt/OptimalTrajectoryGenerator.h>
#include <trajopt/TrajectoryGenerationException.h>
#include <trajopt/trajectory/HolonomicTrajectorySample.h>
#include <trajopt/trajectory/HolonomicTrajectory.h>

#include "trajoptlib/src/lib.rs.h"

namespace trajoptlibrust {

template<typename FromType,
         typename ToType,
         typename FromVecType,
         typename ToVecType,
         ToType(*Converter)(const FromType&)>
ToVecType _convert_generic_vec(const FromVecType& fromVec) {
  ToVecType toVec;
  toVec.reserve(fromVec.size());
  for (const FromType& item : fromVec) {
    toVec.emplace_back(Converter(item));
  }
  return toVec;
}

template<typename RustType,
         typename CppType,
         CppType(*Converter)(const RustType&)>
std::vector<CppType> _rust_vec_to_cpp_vector(const rust::Vec<RustType>& rustVec) {
  return _convert_generic_vec<
      RustType,
      CppType,
      rust::Vec<RustType>,
      std::vector<CppType>,
      Converter>(rustVec);
}

template<typename CppType,
         typename RustType,
         RustType(*Converter)(const CppType&)>
rust::Vec<RustType> _cpp_vector_to_rust_vec(const std::vector<CppType>& cppVec) {
  return _convert_generic_vec<
      CppType,
      RustType,
      std::vector<CppType>,
      rust::Vec<RustType>,
      Converter>(cppVec);
}

trajopt::SwerveModule _convert_swerve_module(const SwerveModule& swerveModule) {
  return trajopt::SwerveModule{
    .x = swerveModule.x,
    .y = swerveModule.y,
    .wheelRadius = swerveModule.wheel_radius,
    .wheelMaxAngularVelocity = swerveModule.wheel_max_angular_velocity,
    .wheelMaxTorque = swerveModule.wheel_max_torque
  };
}

trajopt::SwerveDrivetrain _convert_swerve_drivetrain(const SwerveDrivetrain& drivetrain) {
  return trajopt::SwerveDrivetrain{
    .mass = drivetrain.mass,
    .moi = drivetrain.moi,
    .modules = _rust_vec_to_cpp_vector<SwerveModule, trajopt::SwerveModule, &_convert_swerve_module>(drivetrain.modules)
  };
}

void SwervePathBuilderImpl::set_drivetrain(const SwerveDrivetrain& drivetrain) {
  path.SetDrivetrain(_convert_swerve_drivetrain(drivetrain));
}

void SwervePathBuilderImpl::translation_wpt(size_t idx, double x, double y, double heading_guess) {
  path.TranslationWpt(idx, x, y, heading_guess);
}

void SwervePathBuilderImpl::pose_wpt(size_t idx, double x, double y, double heading) {
  path.PoseWpt(idx, x, y, heading);
}

void SwervePathBuilderImpl::wpt_zero_velocity(size_t idx) {
  path.WptZeroVelocity(idx);
}

void SwervePathBuilderImpl::wpt_zero_angular_velocity(size_t idx) {
  path.WptZeroAngularVelocity(idx);
}

void SwervePathBuilderImpl::wpt_velocity_direction(size_t idx, double angle) {
  path.WptVelocityDirection(idx, angle);
}

void SwervePathBuilderImpl::wpt_velocity_magnitude(size_t idx, double v) {
  path.WptVelocityMagnitude(idx, v);
}

void SwervePathBuilderImpl::wpt_velocity_polar(size_t idx, double vr, double vtheta) {
  path.WptVelocityPolar(idx, vr, vtheta);
}

HolonomicTrajectorySample _convert_holonomic_trajectory_sample(const trajopt::HolonomicTrajectorySample& sample) {
  return HolonomicTrajectorySample{
    .timestamp = sample.timestamp,
    .x = sample.x,
    .y = sample.y,
    .heading = sample.heading,
    .velocity_x = sample.velocityX,
    .velocity_y = sample.velocityY,
    .angular_velocity = sample.angularVelocity,
  };
}

HolonomicTrajectory _convert_holonomic_trajectory(const trajopt::HolonomicTrajectory& trajectory) {
  return HolonomicTrajectory{
    .samples = _cpp_vector_to_rust_vec<
        trajopt::HolonomicTrajectorySample,
        HolonomicTrajectorySample,
        &_convert_holonomic_trajectory_sample>(trajectory.samples)
  };
}

HolonomicTrajectory SwervePathBuilderImpl::generate() const {
  return _convert_holonomic_trajectory(
    trajopt::HolonomicTrajectory{trajopt::OptimalTrajectoryGenerator::Generate(path)});
}

std::unique_ptr<SwervePathBuilderImpl> new_swerve_path_builder_impl() {
  return std::make_unique<SwervePathBuilderImpl>(SwervePathBuilderImpl());
}
}
