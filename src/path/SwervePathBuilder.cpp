// Copyright (c) TrajoptLib contributors

#include "trajopt/path/SwervePathBuilder.hpp"

#include <stdint.h>
#include <wpi/array.h>

#include <cassert>
#include <cmath>
#include <utility>

#include <frc/MathUtil.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryParameterizer.h>

#include "spline/CubicHermitePoseSplineHolonomic.hpp"
#include "spline/SplineParameterizer.hpp"
#include "spline/SplineUtil.hpp"

#include "trajopt/constraint/AngularVelocityMaxMagnitudeConstraint.hpp"
#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/constraint/LinePointConstraint.hpp"
#include "trajopt/constraint/PointLineConstraint.hpp"
#include "trajopt/constraint/PointPointConstraint.hpp"
#include "trajopt/constraint/PoseEqualityConstraint.hpp"
#include "trajopt/constraint/TranslationEqualityConstraint.hpp"
#include "trajopt/obstacle/Obstacle.hpp"
#include "trajopt/solution/SwerveSolution.hpp"
#include "trajopt/util/Cancellation.hpp"
#include "trajopt/util/GenerateLinearInitialGuess.hpp"
#include "trajopt/util/TrajoptUtil.hpp"


namespace trajopt {

void SwervePathBuilder::CancelAll() {
  trajopt::GetCancellationFlag() = 1;
}

SwervePath& SwervePathBuilder::GetPath() {
  return path;
}

void SwervePathBuilder::SetDrivetrain(SwerveDrivetrain drivetrain) {
  path.drivetrain = std::move(drivetrain);
}

void SwervePathBuilder::PoseWpt(size_t index, double x, double y,
                                double heading) {
  WptConstraint(index, PoseEqualityConstraint{x, y, heading});
  WptInitialGuessPoint(index, {x, y, {heading}});
}

void SwervePathBuilder::TranslationWpt(size_t index, double x, double y,
                                       double headingGuess) {
  WptConstraint(index, TranslationEqualityConstraint{x, y});
  WptInitialGuessPoint(index, {x, y, {headingGuess}});
}

void SwervePathBuilder::WptInitialGuessPoint(size_t wptIndex,
                                             const Pose2d& poseGuess) {
  NewWpts(wptIndex);
  initialGuessPoints.at(wptIndex).back() = poseGuess;
}

void SwervePathBuilder::SgmtInitialGuessPoints(
    size_t fromIndex, const std::vector<Pose2d>& sgmtPoseGuess) {
  NewWpts(fromIndex + 1);
  std::vector<Pose2d>& toInitialGuessPoints =
      initialGuessPoints.at(fromIndex + 1);
  toInitialGuessPoints.insert(toInitialGuessPoints.begin(),
                              sgmtPoseGuess.begin(), sgmtPoseGuess.end());
}

void SwervePathBuilder::AddBumpers(Bumpers&& newBumpers) {
  bumpers.emplace_back(std::move(newBumpers));
}

void SwervePathBuilder::WptObstacle(size_t index, const Obstacle& obstacle) {
  for (auto& _bumpers : bumpers) {
    auto minDistance = _bumpers.safetyDistance + obstacle.safetyDistance;

    size_t bumperCornerCount = _bumpers.points.size();
    size_t obstacleCornerCount = obstacle.points.size();
    if (bumperCornerCount == 1 && obstacleCornerCount == 1) {
      // if the bumpers and obstacle are only one point
      WptConstraint(index,
                    PointPointConstraint{_bumpers.points.at(0),
                                         obstacle.points.at(0), minDistance});
      return;
    }

    // robot bumper edge to obstacle point constraints
    for (auto& obstaclePoint : obstacle.points) {
      // First apply constraint for all but last edge
      for (size_t bumperCornerIndex = 0;
           bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
        WptConstraint(index, LinePointConstraint{
                                 _bumpers.points.at(bumperCornerIndex),
                                 _bumpers.points.at(bumperCornerIndex + 1),
                                 obstaclePoint, minDistance});
      }
      // apply to last edge: the edge connecting the last point to the first
      // must have at least three points to need this
      if (bumperCornerCount >= 3) {
        WptConstraint(index,
                      LinePointConstraint{
                          _bumpers.points.at(bumperCornerCount - 1),
                          _bumpers.points.at(0), obstaclePoint, minDistance});
      }
    }

    // obstacle edge to bumper corner constraints
    for (auto& bumperCorner : _bumpers.points) {
      if (obstacleCornerCount > 1) {
        for (size_t obstacleCornerIndex = 0;
             obstacleCornerIndex < obstacleCornerCount - 1;
             obstacleCornerIndex++) {
          WptConstraint(
              index,
              PointLineConstraint{
                  bumperCorner, obstacle.points.at(obstacleCornerIndex),
                  obstacle.points.at(obstacleCornerIndex + 1), minDistance});
        }
        if (obstacleCornerCount >= 3) {
          WptConstraint(index, PointLineConstraint{
                                   bumperCorner,
                                   obstacle.points.at(bumperCornerCount - 1),
                                   obstacle.points.at(0), minDistance});
        }
      } else {
        WptConstraint(index,
                      PointPointConstraint{bumperCorner, obstacle.points.at(0),
                                           minDistance});
      }
    }
  }
}

void SwervePathBuilder::SgmtObstacle(size_t fromIndex, size_t toIndex,
                                     const Obstacle& obstacle) {
  for (auto& _bumpers : bumpers) {
    auto minDistance = _bumpers.safetyDistance + obstacle.safetyDistance;

    size_t bumperCornerCount = _bumpers.points.size();
    size_t obstacleCornerCount = obstacle.points.size();
    if (bumperCornerCount == 1 && obstacleCornerCount == 1) {
      // if the bumpers and obstacle are only one point
      SgmtConstraint(fromIndex, toIndex,
                     PointPointConstraint{_bumpers.points.at(0),
                                          obstacle.points.at(0), minDistance});
      return;
    }

    // robot bumper edge to obstacle point constraints
    for (auto& obstaclePoint : obstacle.points) {
      // First apply constraint for all but last edge
      for (size_t bumperCornerIndex = 0;
           bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
        SgmtConstraint(
            fromIndex, toIndex,
            LinePointConstraint{_bumpers.points.at(bumperCornerIndex),
                                _bumpers.points.at(bumperCornerIndex + 1),
                                obstaclePoint, minDistance});
      }
      // apply to last edge: the edge connecting the last point to the first
      // must have at least three points to need this
      if (bumperCornerCount >= 3) {
        SgmtConstraint(fromIndex, toIndex,
                       LinePointConstraint{
                           _bumpers.points.at(bumperCornerCount - 1),
                           _bumpers.points.at(0), obstaclePoint, minDistance});
      }
    }

    // obstacle edge to bumper corner constraints
    for (auto& bumperCorner : _bumpers.points) {
      if (obstacleCornerCount > 1) {
        for (size_t obstacleCornerIndex = 0;
             obstacleCornerIndex < obstacleCornerCount - 1;
             obstacleCornerIndex++) {
          SgmtConstraint(
              fromIndex, toIndex,
              PointLineConstraint{
                  bumperCorner, obstacle.points.at(obstacleCornerIndex),
                  obstacle.points.at(obstacleCornerIndex + 1), minDistance});
        }
        if (obstacleCornerCount >= 3) {
          SgmtConstraint(
              fromIndex, toIndex,
              PointLineConstraint{bumperCorner,
                                  obstacle.points.at(bumperCornerCount - 1),
                                  obstacle.points.at(0), minDistance});
        }
      } else {
        SgmtConstraint(fromIndex, toIndex,
                       PointPointConstraint{bumperCorner, obstacle.points.at(0),
                                            minDistance});
      }
    }
  }
}

void SwervePathBuilder::ControlIntervalCounts(std::vector<size_t>&& counts) {
  controlIntervalCounts = std::move(counts);
}

const std::vector<size_t>& SwervePathBuilder::GetControlIntervalCounts() const {
  return controlIntervalCounts;
}

SwerveSolution SwervePathBuilder::CalculateInitialGuess() const {
  return GenerateLinearInitialGuess<SwerveSolution>(initialGuessPoints,
                                                    controlIntervalCounts);
}

// TODO make control interval fn that is the first part of the below function
std::vector<frc::Trajectory>
SwervePathBuilder::GenerateWaypointSplineTrajectories() const {
  std::vector<trajopt::CubicHermitePoseSplineHolonomic> splines =
      CubicPoseControlVectorsFromWaypoints(initialGuessPoints);

  // Generate a parameterized spline
  std::vector<std::vector<frc::TrajectoryGenerator::PoseWithCurvature>>
      splinePoints;

  // Iterate through the vector and parameterize each spline
  for (auto&& spline : splines) {
    auto points = trajopt::SplineParameterizer::Parameterize(spline);
    splinePoints.push_back(points);
  }

  const auto maxWheelVelocity = units::meters_per_second_t(
      path.drivetrain.modules.front().wheelMaxAngularVelocity *
      path.drivetrain.modules.front().wheelRadius);

  wpi::array<frc::Translation2d, 4> moduleTranslations{wpi::empty_array};
  for (size_t i = 0; i < path.drivetrain.modules.size(); ++i) {
    const auto mod = path.drivetrain.modules.at(0);
    moduleTranslations.at(0) = 
      frc::Translation2d{units::meter_t(mod.translation.X()), units::meter_t(mod.translation.Y())};
  }
  const frc::SwerveDriveKinematics kinematics{
      moduleTranslations.at(0), moduleTranslations.at(1),
      moduleTranslations.at(2), moduleTranslations.at(3)};

  std::vector<frc::Trajectory> trajs;
  trajs.reserve(path.waypoints.size());
  size_t splineIdx = 0;
  for (size_t sgmtIdx = 1; sgmtIdx < initialGuessPoints.size(); ++sgmtIdx) {
    auto sgmtVel = maxWheelVelocity;
    const auto& sgmtGuessPoints = initialGuessPoints.at(sgmtIdx);
    for (size_t guessIdx = 0; guessIdx < sgmtGuessPoints.size(); ++guessIdx) {
      Pose2d start;
      Pose2d end = sgmtGuessPoints.at(guessIdx);
      if (guessIdx == 0) {
        start = initialGuessPoints.at(sgmtIdx - 1).back();
      } else {
        start = sgmtGuessPoints.at(guessIdx - 1);
      }
      auto dtheta =
          std::abs(frc::AngleModulus(
                      units::radian_t(std::abs(start.Rotation().Radians() - end.Rotation().Radians())))
                       .value());
      frc::Translation2d sgmtStart{units::meter_t(start.Translation().X()),
                                   units::meter_t(start.Translation().Y())};
      frc::Translation2d sgmtEnd{units::meter_t(end.Translation().X()), units::meter_t(end.Translation().Y())};

      for (auto& c : path.waypoints.at(sgmtIdx).segmentConstraints) {
        // assuming HolonomicVelocityConstraint with CircularSet2d
        if (std::holds_alternative<LinearVelocityMaxMagnitudeConstraint>(c)) {
          const auto& velocityHolonomicConstraint =
              std::get<LinearVelocityMaxMagnitudeConstraint>(c);
          auto vel = units::meters_per_second_t(
                        velocityHolonomicConstraint.m_maxMagnitude);
          std::printf("max lin vel: %.2f - ", vel.value());
          if (vel < sgmtVel) {
            sgmtVel = vel;
          }
        } else if (std::holds_alternative<AngularVelocityMaxMagnitudeConstraint>(c)) {
          const auto& angVelConstraint = std::get<AngularVelocityMaxMagnitudeConstraint>(c);
          auto maxAngVel = angVelConstraint.m_maxMagnitude;
          std::printf("max ang vel: %.2f - ", maxAngVel);
          // TODO add how the 1.5 is determined
          auto time = 1.5 * dtheta / maxAngVel;
          // estimating velocity for a straight line path
          // TODO use the spine path distance
          auto vel = sgmtStart.Distance(sgmtEnd) / units::second_t(time);
          if (vel < sgmtVel) {
            sgmtVel = vel;
          }
        }
      }
      std::printf("sgmtVel: %.2f\n", sgmtVel.value());
      frc::TrajectoryConfig sgmtConfig{sgmtVel, sgmtVel / units::second_t{1.0}};
      // uses each non-init guess waypoint as a stop point for first guess
      sgmtConfig.SetStartVelocity(0_mps);
      sgmtConfig.SetEndVelocity(0_mps);
      sgmtConfig.AddConstraint(
          frc::SwerveDriveKinematicsConstraint{kinematics, maxWheelVelocity});

      // specify parameterized spline points to use for trajectory
      const auto sgmtTraj = frc::TrajectoryParameterizer::
          TrajectoryParameterizer::TimeParameterizeTrajectory(
              splinePoints.at(++splineIdx - 1), sgmtConfig.Constraints(),
              sgmtConfig.StartVelocity(), sgmtConfig.EndVelocity(),
              sgmtConfig.MaxVelocity(), sgmtConfig.MaxAcceleration(),
              sgmtConfig.IsReversed());
      trajs.push_back(sgmtTraj);
    }
  }
  std::printf("path.wpt size: %zd\n", path.waypoints.size());
  std::printf("trajs size: %zd\n", trajs.size());
  return trajs;
}

std::vector<std::vector<frc::Trajectory::State>>
SwervePathBuilder::CalculateWaypointStatesWithDt(const double desiredDt) const {
  const auto trajs = GenerateWaypointSplineTrajectories();

  size_t guessPoints = 0;
  for (const auto& guesses : initialGuessPoints) {
    guessPoints += guesses.size();
  }
  std::vector<std::vector<frc::Trajectory::State>> waypoint_states;
  waypoint_states.reserve(guessPoints);
  for (size_t i = 0; i < guessPoints; ++i) {
    waypoint_states.push_back(std::vector<frc::Trajectory::State>());
  }

  for (size_t sgmtIdx = 1; sgmtIdx < guessPoints; ++sgmtIdx) {
    // specify parameterized spline points to use for trajectory
    const auto sgmtTraj = trajs.at(sgmtIdx - 1);

    const auto wholeSgmtDt = sgmtTraj.TotalTime();
    const size_t samplesForSgmtNew = std::ceil(wholeSgmtDt.value() / desiredDt);
    const auto dt = wholeSgmtDt / samplesForSgmtNew;
    std::printf("dt for sgmt%zd with %zd samples: %.5f\n", sgmtIdx,
                samplesForSgmtNew, dt.value());

    if (sgmtIdx == 1) {
      std::printf("sgmt1\n");
      waypoint_states.at(sgmtIdx - 1).push_back(sgmtTraj.States().front());
    }

    for (size_t sampleIdx = 1; sampleIdx <= samplesForSgmtNew; ++sampleIdx) {
      auto t = static_cast<double>(sampleIdx) * dt;
      const auto point = sgmtTraj.Sample(t);
      waypoint_states.at(sgmtIdx).push_back(point);
      std::printf("%zd,", sampleIdx);
    }
    std::printf(" size: %zd\n", waypoint_states.at(sgmtIdx).size());
  }
  return waypoint_states;
}

std::vector<std::vector<frc::Trajectory::State>>
SwervePathBuilder::CalculateWaypointStatesWithControlIntervals() const {
  const auto trajs = GenerateWaypointSplineTrajectories();

  size_t guessPoints = 0;
  for (const auto& guesses : initialGuessPoints) {
    guessPoints += guesses.size();
  }
  std::vector<std::vector<frc::Trajectory::State>> waypoint_states;
  waypoint_states.reserve(guessPoints);
  for (size_t i = 0; i < guessPoints; ++i) {
    waypoint_states.push_back(std::vector<frc::Trajectory::State>());
  }

  size_t trajIdx = 0;
  std::printf("sgmt1\n");
  waypoint_states.at(0).push_back(trajs.at(trajIdx).States().front());
  std::printf("ctrlCount: [");
  for (auto count : controlIntervalCounts) {
    std::printf("%zd,", count);
  }
  std::printf("]\n");
  for (size_t sgmtIdx = 1; sgmtIdx < initialGuessPoints.size(); ++sgmtIdx) {
    auto guessPointsSize = initialGuessPoints.at(sgmtIdx).size();
    auto samplesForSgmt = controlIntervalCounts.at(sgmtIdx - 1);
    size_t samples = samplesForSgmt / guessPointsSize;
    for (size_t guessIdx = 0; guessIdx < guessPointsSize; ++guessIdx) {
      if (guessIdx == (guessPointsSize - 1)) {
        samples += (samplesForSgmt % guessPointsSize);
      }
      for (size_t sampleIdx = 0; sampleIdx < samples; ++sampleIdx) {
        auto t = trajs.at(trajIdx).TotalTime() *
                 static_cast<double>(sampleIdx) / samples;
        const auto state = trajs.at(trajIdx).Sample(t);
        waypoint_states.at(trajIdx + 1).push_back(state);
        std::printf("%zd,", sampleIdx);
      }
      std::printf(" size: %zd\n", waypoint_states.at(trajIdx + 1).size());
      ++trajIdx;
    }
  }
  return waypoint_states;
}

std::vector<size_t> SwervePathBuilder::CalculateControlIntervalCounts() const {
  const auto desiredDt = 0.1;
  const auto trajectoriesSamples = CalculateWaypointStatesWithDt(desiredDt);
  std::vector<size_t> counts;
  counts.reserve(path.waypoints.size());
  for (size_t i = 1; i < trajectoriesSamples.size(); ++i) {
    counts.push_back(trajectoriesSamples.at(i).size());
  }
  counts.push_back(1);
  return counts;
}

SwerveSolution
SwervePathBuilder::CalculateSplineInitialGuessWithKinematicsAndConstraints()
    const {
  const auto trajectoriesSamples =
      CalculateWaypointStatesWithControlIntervals();

  SwerveSolution initialGuess{};
  for (const auto& traj : trajectoriesSamples) {
    auto dt = 0.1_s;
    if (traj.size() > 1) {
      dt = traj.at(1).t - traj.front().t;
    }
    for (const auto& point : traj) {
      initialGuess.x.push_back(point.pose.X().value());
      initialGuess.y.push_back(point.pose.Y().value());
      initialGuess.thetacos.push_back(point.pose.Rotation().Cos());
      initialGuess.thetasin.push_back(point.pose.Rotation().Sin());
      initialGuess.dt.push_back(dt.value());
    }
  }

  // fix headings 
  /// FIXME: TODO: NOT SURE IF THIS IS NEEDED AFTER THE SIN/COS CHANGE
  /*
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
  */

  return initialGuess;
}

void SwervePathBuilder::AddIntermediateCallback(
    const std::function<void(SwerveSolution&, int64_t)> callback) {
  path.callbacks.push_back(callback);
}

void SwervePathBuilder::NewWpts(size_t finalIndex) {
  int64_t targetIndex = finalIndex;
  int64_t greatestIndex = path.waypoints.size() - 1;
  if (targetIndex > greatestIndex) {
    for (int64_t i = greatestIndex + 1; i <= targetIndex; ++i) {
      path.waypoints.emplace_back();
      initialGuessPoints.emplace_back(std::vector<Pose2d>{{0.0, 0.0, {0.0}}});
      if (i != 0) {
        controlIntervalCounts.push_back(40);
      }
    }
  }
}

}  // namespace trajopt
