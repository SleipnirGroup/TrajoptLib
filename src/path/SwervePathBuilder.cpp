// Copyright (c) TrajoptLib contributors

#include "trajopt/path/SwervePathBuilder.h"

#include <frc/MathUtil.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryParameterizer.h>
#include <stdint.h>
#include <wpi/array.h>

#include <cassert>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "optimization/Cancellation.h"
#include "optimization/TrajoptUtil.h"
#include "spline/CubicHermitePoseSplineHolonomic.h"
#include "spline/SplineParameterizer.h"
#include "spline/SplineUtil.h"
#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/path/Path.h"
#include "trajopt/set/EllipticalSet2d.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/LinearSet2d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/solution/Solution.h"

namespace trajopt {

const SwervePath& SwervePathBuilder::GetPath() const {
  return path;
}

void SwervePathBuilder::SetDrivetrain(SwerveDrivetrain drivetrain) {
  path.drivetrain = std::move(drivetrain);
}

void SwervePathBuilder::PoseWpt(size_t index, double x, double y,
                                double heading) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
      TranslationConstraint{RectangularSet2d{x, y}});
  path.waypoints.at(index).waypointConstraints.emplace_back(
      HeadingConstraint{heading});
  WptInitialGuessPoint(index, InitialGuessPoint{x, y, heading});
}

void SwervePathBuilder::TranslationWpt(size_t index, double x, double y,
                                       double headingGuess) {
  NewWpts(index);
  path.waypoints.at(index).waypointConstraints.emplace_back(
      TranslationConstraint{RectangularSet2d{x, y}});
  WptInitialGuessPoint(index, InitialGuessPoint{x, y, headingGuess});
}

void SwervePathBuilder::WptInitialGuessPoint(
    size_t wptIdx, const InitialGuessPoint& poseGuess) {
  NewWpts(wptIdx);
  initialGuessPoints.at(wptIdx).back() = poseGuess;
}

void SwervePathBuilder::SgmtInitialGuessPoints(
    size_t fromIdx, const std::vector<InitialGuessPoint>& sgmtPoseGuess) {
  NewWpts(fromIdx + 1);
  std::vector<InitialGuessPoint>& toInitialGuessPoints =
      initialGuessPoints.at(fromIdx + 1);
  toInitialGuessPoints.insert(toInitialGuessPoints.begin(),
                              sgmtPoseGuess.begin(), sgmtPoseGuess.end());
}

void SwervePathBuilder::WptVelocityDirection(size_t idx, double angle) {
  WptConstraint(idx, HolonomicVelocityConstraint{LinearSet2d{angle},
                                                 CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityMagnitude(size_t idx, double v) {
  if (std::abs(v) < 1e-4) {
    WptZeroVelocity(idx);
  } else {
    WptConstraint(idx,
                  HolonomicVelocityConstraint{EllipticalSet2d::CircularSet2d(v),
                                              CoordinateSystem::kField});
  }
}

void SwervePathBuilder::WptZeroVelocity(size_t idx) {
  WptConstraint(idx, HolonomicVelocityConstraint{RectangularSet2d{0.0, 0.0},
                                                 CoordinateSystem::kField});
}

void SwervePathBuilder::WptVelocityPolar(size_t idx, double vr, double vtheta) {
  WptConstraint(idx, HolonomicVelocityConstraint{
                         RectangularSet2d::PolarExactSet2d(vr, vtheta),
                         CoordinateSystem::kField});
}

void SwervePathBuilder::WptAngularVelocity(size_t idx,
                                           double angular_velocity) {
  WptConstraint(idx, AngularVelocityConstraint{angular_velocity});
}

void SwervePathBuilder::WptAngularVelocityMaxMagnitude(
    size_t idx, double angular_velocity) {
  WptConstraint(idx, AngularVelocityConstraint{
                         IntervalSet1d{-angular_velocity, angular_velocity}});
}

void SwervePathBuilder::WptZeroAngularVelocity(size_t idx) {
  WptConstraint(idx, AngularVelocityConstraint{0.0});
}

void SwervePathBuilder::SgmtVelocityDirection(size_t fromIdx, size_t toIdx,
                                              double angle, bool includeWpts) {
  SgmtConstraint(
      fromIdx, toIdx,
      HolonomicVelocityConstraint{LinearSet2d{angle}, CoordinateSystem::kField},
      includeWpts);
}

void SwervePathBuilder::SgmtVelocityMagnitude(size_t fromIdx, size_t toIdx,
                                              double v, bool includeWpts) {
  Set2d set = EllipticalSet2d{v, v, EllipticalSet2d::Direction::kInside};
  if (std::abs(v) < 1e-4) {
    set = RectangularSet2d{0.0, 0.0};
  }
  SgmtConstraint(fromIdx, toIdx,
                 HolonomicVelocityConstraint{set, CoordinateSystem::kField},
                 includeWpts);
}

void SwervePathBuilder::SgmtAngularVelocity(size_t fromIdx, size_t toIdx,
                                            double angular_velocity,
                                            bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx, AngularVelocityConstraint{angular_velocity},
                 includeWpts);
}

void SwervePathBuilder::SgmtAngularVelocityMaxMagnitude(size_t fromIdx,
                                                        size_t toIdx,
                                                        double angular_velocity,
                                                        bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx,
                 AngularVelocityConstraint{
                     IntervalSet1d{-angular_velocity, angular_velocity}},
                 includeWpts);
}

void SwervePathBuilder::SgmtZeroAngularVelocity(size_t fromIdx, size_t toIdx,
                                                bool includeWpts) {
  SgmtConstraint(fromIdx, toIdx, AngularVelocityConstraint{0.0}, includeWpts);
}

void SwervePathBuilder::WptConstraint(size_t idx,
                                      const HolonomicConstraint& constraint) {
  NewWpts(idx);
  path.waypoints.at(idx).waypointConstraints.push_back(constraint);
}

void SwervePathBuilder::SgmtConstraint(size_t fromIdx, size_t toIdx,
                                       const HolonomicConstraint& constraint,
                                       bool includeWpts) {
  assert(fromIdx < toIdx);

  NewWpts(toIdx);
  if (includeWpts) {
    path.waypoints.at(fromIdx).waypointConstraints.push_back(constraint);
  }
  for (size_t idx = fromIdx + 1; idx <= toIdx; idx++) {
    if (includeWpts) {
      path.waypoints.at(idx).waypointConstraints.push_back(constraint);
    }
    path.waypoints.at(idx).segmentConstraints.push_back(constraint);
  }
}

void SwervePathBuilder::AddBumpers(Bumpers&& newBumpers) {
  bumpers.emplace_back(std::move(newBumpers));
}

void SwervePathBuilder::WptObstacle(size_t idx, const Obstacle& obstacle) {
  for (auto& _bumpers : bumpers) {
    auto constraints = GetConstraintsForObstacle(_bumpers, obstacle);
    for (auto& constraint : constraints) {
      WptConstraint(idx, constraint);
    }
  }
}

void SwervePathBuilder::SgmtObstacle(size_t fromIdx, size_t toIdx,
                                     const Obstacle& obstacle,
                                     bool includeWpts) {
  for (auto& _bumpers : bumpers) {
    auto constraints = GetConstraintsForObstacle(_bumpers, obstacle);
    for (auto& constraint : constraints) {
      SgmtConstraint(fromIdx, toIdx, constraint, includeWpts);
    }
  }
}

void SwervePathBuilder::ControlIntervalCounts(std::vector<size_t>&& counts) {
  controlIntervalCounts = std::move(counts);
}

const std::vector<size_t>& SwervePathBuilder::GetControlIntervalCounts() const {
  return controlIntervalCounts;
}

Solution SwervePathBuilder::CalculateLinearInitialGuess() const {
  return GenerateLinearInitialGuess(initialGuessPoints, controlIntervalCounts);
}

// TODO make control interval fn that is the first part of the below function
std::vector<std::vector<frc::Trajectory::State>>
SwervePathBuilder::CalculateWaypointStates() const {
  std::vector<trajopt::CubicHermitePoseSplineHolonomic> splines =
      CubicPoseControlVectorsFromWaypoints(initialGuessPoints);

  // Generate a parameterized spline
  std::vector<std::vector<frc::TrajectoryGenerator::PoseWithCurvature>>
      splinePoints;

  // Iterate through the vector and parameterize each spline
  for (auto&& spline : splines) {
    auto points = SplineParameterizer::Parameterize(spline);
    splinePoints.push_back(points);
  }

  const auto maxWheelVelocity = units::meters_per_second_t(
      path.drivetrain.modules.front().wheelMaxAngularVelocity *
      path.drivetrain.modules.front().wheelRadius);

  wpi::array<frc::Translation2d, 4> moduleTranslations{wpi::empty_array};
  for (size_t i = 0; i < path.drivetrain.modules.size(); ++i) {
    const auto mod = path.drivetrain.modules.at(0);
    moduleTranslations.at(0) =
        frc::Translation2d{units::meter_t(mod.x), units::meter_t(mod.y)};
  }
  const frc::SwerveDriveKinematics kinematics{
      moduleTranslations.at(0), moduleTranslations.at(1),
      moduleTranslations.at(2), moduleTranslations.at(3)};

  std::vector<std::vector<frc::Trajectory::State>> waypoint_states;
  waypoint_states.reserve(path.waypoints.size());
  for (size_t i = 0; i < path.waypoints.size(); ++i) {
    waypoint_states.push_back(std::vector<frc::Trajectory::State>());
  }
  std::vector<size_t> controlIntervalCountsSpline;
  controlIntervalCountsSpline.reserve(controlIntervalCounts.size());
  for (size_t sgmtIdx = 1; sgmtIdx < path.waypoints.size(); ++sgmtIdx) {
    auto sgmtVel = maxWheelVelocity;
    auto dtheta = frc::AngleModulus(units::radian_t(
        std::abs(initialGuessPoints.at(sgmtIdx - 1).back().heading -
                 initialGuessPoints.at(sgmtIdx).back().heading)));
    frc::Translation2d sgmtStart{
        units::meter_t(initialGuessPoints.at(sgmtIdx - 1).back().x),
        units::meter_t(initialGuessPoints.at(sgmtIdx - 1).back().y)};
    frc::Translation2d sgmtEnd{
        units::meter_t(initialGuessPoints.at(sgmtIdx).back().x),
        units::meter_t(initialGuessPoints.at(sgmtIdx).back().y)};
    for (auto& c : path.waypoints.at(sgmtIdx).segmentConstraints) {
      // assuming HolonomicVelocityConstraint with CircularSet2d
      if (std::holds_alternative<HolonomicVelocityConstraint>(c)) {
        const auto& velocityHolonomicConstraint =
            std::get<HolonomicVelocityConstraint>(c);
        auto set2d = velocityHolonomicConstraint.velocityBound;
        if (std::holds_alternative<EllipticalSet2d>(set2d)) {
          auto vel = units::meters_per_second_t(
              std::abs(std::get<EllipticalSet2d>(set2d).xRadius));
          if (vel < sgmtVel) {
            sgmtVel = vel;
          }
        }
      } else if (std::holds_alternative<AngularVelocityConstraint>(c)) {
        const auto& angVelConstraint = std::get<AngularVelocityConstraint>(c);
        auto maxAngVel = std::abs(angVelConstraint.angularVelocityBound.upper);
        // TODO add how the 1.5 is determined
        auto time = 1.5 * dtheta.value() / maxAngVel;
        // estimating velocity for a straight line path
        // TODO use the spine path distance
        sgmtVel = sgmtStart.Distance(sgmtEnd) / units::second_t(time);
      }
    }

    frc::TrajectoryConfig sgmtConfig{sgmtVel, sgmtVel / units::second_t{1.0}};
    // uses each non-init guess waypoint as a stop point for first guess
    sgmtConfig.SetStartVelocity(0_mps);
    sgmtConfig.SetEndVelocity(0_mps);
    sgmtConfig.AddConstraint(
        frc::SwerveDriveKinematicsConstraint{kinematics, maxWheelVelocity});

    // specify parameterized spline points to use for trajectory
    const auto sgmtTraj = frc::TrajectoryParameterizer::
        TrajectoryParameterizer::TimeParameterizeTrajectory(
            splinePoints.at(sgmtIdx - 1), sgmtConfig.Constraints(),
            sgmtConfig.StartVelocity(), sgmtConfig.EndVelocity(),
            sgmtConfig.MaxVelocity(), sgmtConfig.MaxAcceleration(),
            sgmtConfig.IsReversed());

    const auto wholeSgmtDt = sgmtTraj.TotalTime();
    const auto desiredDt = 0.1;
    const size_t samplesForSgmtNew = std::ceil(wholeSgmtDt.value() / desiredDt);
    controlIntervalCountsSpline.push_back(samplesForSgmtNew);
    const auto dt = wholeSgmtDt / samplesForSgmtNew;
    std::printf("dt for sgmt%zd with %zd samples: %.5f\n", sgmtIdx,
                samplesForSgmtNew, dt.value());

    if (sgmtIdx == 1) {
      waypoint_states.at(sgmtIdx - 1).push_back(sgmtTraj.States().front());
    }
    for (size_t sampleIdx = 1; sampleIdx <= samplesForSgmtNew; ++sampleIdx) {
      auto t = static_cast<double>(sampleIdx) * dt;
      const auto point = sgmtTraj.Sample(t);
      waypoint_states.at(sgmtIdx).push_back(point);
    }
  }
  return waypoint_states;
  // return std::vector<std::vector<size_t>>{std::vector<size_t>{1,2,3,4}};
}

std::vector<size_t> SwervePathBuilder::CalculateControlIntervalCounts() const {
  const auto trajs = CalculateWaypointStates();
  std::vector<size_t> counts;
  counts.reserve(path.waypoints.size());
  for (size_t i = 1; i < trajs.size(); ++i) {
    counts.push_back(trajs.at(i).size());
  }
  counts.push_back(1);
  return counts;
}

Solution
SwervePathBuilder::CalculateSplineInitialGuessWithKinematicsAndConstraints()
    const {
  const auto trajs = CalculateWaypointStates();
  // const std::vector<std::vector<frc::Trajectory::State>> trajs{
  //   std::vector<frc::Trajectory::State>{
  //     frc::Trajectory::State{0_s, 0_mps, 0_mps_sq,
  //       frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_rad}},
  //       units::curvature_t{0.0}}
  //   }
  // };

  Solution initialGuess{};
  for (const auto& traj : trajs) {
    auto dt = 0.1_s;
    if (traj.size() > 1) {
      dt = traj.at(1).t - traj.front().t;
    }
    for (const auto& point : traj) {
      initialGuess.x.push_back(point.pose.X().value());
      initialGuess.y.push_back(point.pose.Y().value());
      initialGuess.theta.push_back(point.pose.Rotation().Radians().value());
      initialGuess.dt.push_back(dt.value());
    }
  }

  // fix headings
  int fullRots = 0;
  double prevHeading = initialGuess.theta.front();
  for (size_t i = 0; i < initialGuess.theta.size(); ++i) {
    const auto prevHeadingMod =
        frc::AngleModulus(units::radian_t(prevHeading)).value();
    const auto heading = initialGuess.theta.at(i);
    const auto headingMod = frc::AngleModulus(units::radian_t(heading)).value();
    if (prevHeadingMod < 0 && headingMod > prevHeadingMod + std::numbers::pi) {
      fullRots--;
    } else if (prevHeadingMod > 0 &&
               heading < prevHeadingMod - std::numbers::pi) {
      fullRots++;
    }
    initialGuess.theta.at(i) = fullRots * 2.0 * std::numbers::pi + headingMod;
    prevHeading = initialGuess.theta.at(i);
  }

  // double totalTime = 0.0;
  // std::printf("init solution: [\n");
  // for (size_t i = 0; i < initialGuess.x.size(); ++i) {
  //   totalTime += initialGuess.dt.at(i);
  //   std::printf("[timestamp: %.3f, x: %.3f, y: %.3f, theta: %.3f]\n",
  //   totalTime,
  //               initialGuess.x.at(i), initialGuess.y.at(i),
  //               initialGuess.theta.at(i));
  // }
  // std::printf("]\n");

  return initialGuess;
}

void SwervePathBuilder::NewWpts(size_t finalIndex) {
  int64_t targetIdx = finalIndex;
  int64_t greatestIdx = path.waypoints.size() - 1;
  if (targetIdx > greatestIdx) {
    for (int64_t i = greatestIdx + 1; i <= targetIdx; ++i) {
      path.waypoints.emplace_back(SwerveWaypoint{});
      initialGuessPoints.emplace_back(
          std::vector{InitialGuessPoint{0.0, 0.0, 0.0}});
      if (i != 0) {
        controlIntervalCounts.push_back(40);
      }
    }
  }
}

std::vector<HolonomicConstraint> SwervePathBuilder::GetConstraintsForObstacle(
    const Bumpers& bumpers, const Obstacle& obstacle) {
  auto distConst = IntervalSet1d::GreaterThan(bumpers.safetyDistance +
                                              obstacle.safetyDistance);

  size_t bumperCornerCount = bumpers.points.size();
  size_t obstacleCornerCount = obstacle.points.size();
  if (bumperCornerCount == 1 && obstacleCornerCount == 1) {
    // if the bumpers and obstacle are only one point
    return {PointPointConstraint{bumpers.points.at(0).x, bumpers.points.at(0).y,
                                 obstacle.points.at(0).x,
                                 obstacle.points.at(0).y, distConst}};
  }

  std::vector<HolonomicConstraint> constraints;

  // robot bumper edge to obstacle point constraints
  for (auto& obstaclePoint : obstacle.points) {
    // First apply constraint for all but last edge
    for (size_t bumperCornerIndex = 0;
         bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
      constraints.emplace_back(
          LinePointConstraint{bumpers.points.at(bumperCornerIndex).x,
                              bumpers.points.at(bumperCornerIndex).y,
                              bumpers.points.at(bumperCornerIndex + 1).x,
                              bumpers.points.at(bumperCornerIndex + 1).y,
                              obstaclePoint.x, obstaclePoint.y, distConst});
    }
    // apply to last edge: the edge connecting the last point to the first
    // must have at least three points to need this
    if (bumperCornerCount >= 3) {
      constraints.emplace_back(LinePointConstraint{
          bumpers.points.at(bumperCornerCount - 1).x,
          bumpers.points.at(bumperCornerCount - 1).y, bumpers.points.at(0).x,
          bumpers.points.at(0).y, obstaclePoint.x, obstaclePoint.y, distConst});
    }
  }

  // obstacle edge to bumper corner constraints
  for (auto& bumperCorner : bumpers.points) {
    if (obstacleCornerCount > 1) {
      for (size_t obstacleCornerIndex = 0;
           obstacleCornerIndex < obstacleCornerCount - 1;
           obstacleCornerIndex++) {
        constraints.emplace_back(PointLineConstraint{
            bumperCorner.x, bumperCorner.y,
            obstacle.points.at(obstacleCornerIndex).x,
            obstacle.points.at(obstacleCornerIndex).y,
            obstacle.points.at(obstacleCornerIndex + 1).x,
            obstacle.points.at(obstacleCornerIndex + 1).y, distConst});
      }
      if (obstacleCornerCount >= 3) {
        constraints.emplace_back(PointLineConstraint{
            bumperCorner.x, bumperCorner.y,
            obstacle.points.at(bumperCornerCount - 1).x,
            obstacle.points.at(bumperCornerCount - 1).y,
            obstacle.points.at(0).x, obstacle.points.at(0).y, distConst});
      }
    } else {
      constraints.emplace_back(PointPointConstraint{
          bumperCorner.x, bumperCorner.y, obstacle.points.at(0).x,
          obstacle.points.at(0).y, distConst});
    }
  }
  return constraints;
}

void SwervePathBuilder::AddIntermediateCallback(
    const std::function<void(SwerveSolution&, int64_t)> callback) {
  path.callbacks.push_back(callback);
}

void SwervePathBuilder::CancelAll() {
  trajopt::GetCancellationFlag() = 1;
}
}  // namespace trajopt
