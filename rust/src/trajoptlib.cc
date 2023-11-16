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

trajopt::InitialGuessPoint _convert_initial_guess_point(const InitialGuessPoint& initialGuessPoint) {
  return trajopt::InitialGuessPoint{
    .x = initialGuessPoint.x,
    .y = initialGuessPoint.y,
    .heading = initialGuessPoint.heading
  };
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
    _rust_vec_to_cpp_vector<size_t, size_t, &_convert_count>
    (counts);
  path.ControlIntervalCounts(std::move(converted_counts));
}

void SwervePathBuilderImpl::set_bumpers(double length, double width) {
  path.AddBumpers(trajopt::Bumpers{
    .safetyDistance = 0.0,
    .points = {
      {+length / 2, +width / 2},
      {-length / 2, +width / 2},
      {-length / 2, -width / 2},
      {+length / 2, -width / 2}
    }
  });
}

void SwervePathBuilderImpl::pose_wpt(size_t idx, double x, double y, double heading) {
  path.PoseWpt(idx, x, y, heading);
}

void SwervePathBuilderImpl::translation_wpt(size_t idx, double x, double y, double heading_guess) {
  path.TranslationWpt(idx, x, y, heading_guess);
}

void SwervePathBuilderImpl::empty_wpt(size_t idx, double x_guess, double y_guess, double heading_guess) {
  path.WptInitialGuessPoint(idx, {x_guess, y_guess, heading_guess});
}

void SwervePathBuilderImpl::sgmt_initial_guess_points(
  size_t from_idx, const rust::Vec<InitialGuessPoint>& guess_points) {
  std::vector<trajopt::InitialGuessPoint> convertedGuessPoints =
      _rust_vec_to_cpp_vector<InitialGuessPoint,
                              trajopt::InitialGuessPoint,
                              &_convert_initial_guess_point>(guess_points);
  path.SgmtInitialGuessPoints(from_idx, convertedGuessPoints);
}

void SwervePathBuilderImpl::wpt_linear_velocity_direction(size_t idx, double angle) {
  path.WptVelocityDirection(idx, angle);
}

void SwervePathBuilderImpl::wpt_linear_velocity_max_magnitude(size_t idx, double magnitude) {
  path.WptVelocityMagnitude(idx, magnitude);
}

void SwervePathBuilderImpl::wpt_linear_velocity_polar(size_t idx, double magnitude, double angle) {
  path.WptVelocityPolar(idx, magnitude, angle);
}

void SwervePathBuilderImpl::wpt_angular_velocity(size_t idx, double angular_velocity) {
  // this probably ought to be added to SwervePathBuilder in the C++ API
  path.WptConstraint(idx, trajopt::AngularVelocityConstraint{angular_velocity});
}

void SwervePathBuilderImpl::wpt_x(size_t idx, double x) {
  path.WptConstraint(idx, trajopt::TranslationConstraint{
    trajopt::RectangularSet2d{
      .xBound = x,
      .yBound = trajopt::IntervalSet1d::R1()
    }
  });
}

void SwervePathBuilderImpl::wpt_y(size_t idx, double y) {
  path.WptConstraint(idx, trajopt::TranslationConstraint{
    trajopt::RectangularSet2d{
      .xBound = trajopt::IntervalSet1d::R1(),
      .yBound = y
    }
  });
}

void SwervePathBuilderImpl::wpt_heading(size_t idx, double heading) {
  path.WptConstraint(idx, trajopt::HeadingConstraint{
    trajopt::IntervalSet1d(heading)
  });
}

void SwervePathBuilderImpl::sgmt_linear_velocity_direction(size_t from_idx, size_t to_idx, double angle) {
  path.SgmtVelocityDirection(from_idx, to_idx, angle);
}

void SwervePathBuilderImpl::sgmt_linear_velocity_max_magnitude(size_t from_idx, size_t to_idx, double magnitude) {
  path.SgmtVelocityMagnitude(from_idx, to_idx, magnitude);
}

void SwervePathBuilderImpl::sgmt_linear_velocity_polar(size_t from_idx, size_t to_idx, double magnitude, double angle) {
  path.SgmtConstraint(from_idx, to_idx, trajopt::HolonomicVelocityConstraint{
    trajopt::RectangularSet2d::PolarExactSet2d(magnitude, angle),
    trajopt::CoordinateSystem::kField
  });
}

void SwervePathBuilderImpl::sgmt_angular_velocity(size_t from_idx, size_t to_idx, double angular_velocity) {
  path.SgmtConstraint(from_idx, to_idx, trajopt::AngularVelocityConstraint{angular_velocity});
}

void SwervePathBuilderImpl::sgmt_x(size_t from_idx, size_t to_idx, double x) {
  path.SgmtConstraint(from_idx, to_idx, trajopt::TranslationConstraint{
    trajopt::RectangularSet2d{
      .xBound = x,
      .yBound = trajopt::IntervalSet1d::R1()
    }
  });
}

void SwervePathBuilderImpl::sgmt_y(size_t from_idx, size_t to_idx, double y) {
  path.SgmtConstraint(from_idx, to_idx, trajopt::TranslationConstraint{
    trajopt::RectangularSet2d{
      .xBound = trajopt::IntervalSet1d::R1(),
      .yBound = y
    }
  });
}

void SwervePathBuilderImpl::sgmt_heading(size_t from_idx, size_t to_idx, double heading) {
  path.SgmtConstraint(from_idx, to_idx, trajopt::HeadingConstraint{heading});
}

void SwervePathBuilderImpl::sgmt_circle_obstacle(size_t from_idx, size_t to_idx, double x, double y, double radius) {
  auto obstacle = trajopt::Obstacle{
    .safetyDistance = radius,
    .points = {{x, y}}
  };
  path.SgmtObstacle(from_idx, to_idx, obstacle);
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

void SwervePathBuilderImpl::cancel_all() {
  path.CancelAll();
}
}
