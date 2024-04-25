// Copyright (c) TrajoptLib contributors

#include "trajopt/path/SwervePathBuilder.h"

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
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

Solution SwervePathBuilder::CalculateSplineInitialGuess() const {
  return GenerateSplineInitialGuess(initialGuessPoints, controlIntervalCounts,
                                    path);
}

Solution SwervePathBuilder::CalculateSplineInitialGuessWithKinematics() const {
  size_t totalGuessPoints = 0;
  for (const auto& points : initialGuessPoints) {
    totalGuessPoints += points.size();
  }
  std::vector<frc::Translation2d> flatTranslationPoints;
  std::vector<frc::Rotation2d> flatHeadings;
  flatTranslationPoints.reserve(totalGuessPoints);
  flatHeadings.reserve(totalGuessPoints);

  // populate translation and heading vectors
  for (const auto& guessPoints : initialGuessPoints) {
    for (const auto& guessPoint : guessPoints) {
      flatTranslationPoints.emplace_back(units::meter_t(guessPoint.x),
                                         units::meter_t(guessPoint.y));
      flatHeadings.emplace_back(units::radian_t(guessPoint.heading));
    }
  }

  // calculate angles and pose for start and end of path spline
  const auto startSplineAngle =
      (flatTranslationPoints.at(1) - flatTranslationPoints.at(0)).Angle();
  const auto endSplineAngle =
      (flatTranslationPoints.back() -
       flatTranslationPoints.at(flatTranslationPoints.size() - 2))
          .Angle();
  const frc::Pose2d start{flatTranslationPoints.front(), startSplineAngle};
  const frc::Pose2d end{flatTranslationPoints.back(), endSplineAngle};

  // use all interior points to create the path spline
  std::vector<frc::Translation2d> interiorPoints{
      flatTranslationPoints.begin() + 1, flatTranslationPoints.end() - 1};

  const auto splineControlVectors =
      frc::SplineHelper::CubicControlVectorsFromWaypoints(start, interiorPoints,
                                                          end);
  const auto splines_temp = frc::SplineHelper::CubicSplinesFromControlVectors(
      splineControlVectors.front(), interiorPoints,
      splineControlVectors.back());

  std::vector<trajopt::CubicHermitePoseSplineHolonomic> splines;
  splines.reserve(splines_temp.size());
  for (size_t i = 1; i <= splines_temp.size(); ++i) {
    splines.emplace_back(splines_temp.at(i - 1), flatHeadings.at(i - 1),
                         flatHeadings.at(i));
  }

  // Generate a parameterized spline
  std::vector<frc::TrajectoryGenerator::PoseWithCurvature> splinePoints;
  std::vector<size_t> pointsPerSpline;
  pointsPerSpline.reserve(splines.size());

  // Add the first point to the vector.
  splinePoints.push_back(splines.front().GetPoint(0.0));

  // Iterate through the vector and parameterize each spline, adding the
  // parameterized points to the final vector.
  for (auto&& spline : splines) {
    auto points = SplineParameterizer::Parameterize(spline);
    // Append the array of poses to the vector.
    // Remove the first point because it's a duplicate
    // of the last point from the previous spline.
    splinePoints.insert(std::end(splinePoints), std::begin(points) + 1,
                        std::end(points));
    pointsPerSpline.push_back(points.size());
  }

  const auto maxWheelVelocity = units::meters_per_second_t(
      path.drivetrain.modules.front().wheelMaxAngularVelocity *
      path.drivetrain.modules.front().wheelRadius);
  frc::TrajectoryConfig config{
      maxWheelVelocity,
      units::meters_per_second_squared_t(maxWheelVelocity.value())};
  config.SetStartVelocity(units::meters_per_second_t(0));
  config.SetEndVelocity(units::meters_per_second_t(0));

  wpi::array<frc::Translation2d, 4> moduleTranslations{wpi::empty_array};
  for (size_t i = 0; i < path.drivetrain.modules.size(); ++i) {
    const auto mod = path.drivetrain.modules.at(0);
    moduleTranslations.at(0) =
        frc::Translation2d{units::meter_t(mod.x), units::meter_t(mod.y)};
  }

  frc::SwerveDriveKinematics kinematics{
      moduleTranslations.at(0), moduleTranslations.at(1),
      moduleTranslations.at(2), moduleTranslations.at(3)};
  config.AddConstraint(
      frc::SwerveDriveKinematicsConstraint{kinematics, maxWheelVelocity});

  // time parameterize
  const auto traj = frc::TrajectoryParameterizer::TrajectoryParameterizer::
      TimeParameterizeTrajectory(splinePoints, config.Constraints(),
                                 config.StartVelocity(), config.EndVelocity(),
                                 config.MaxVelocity(), config.MaxAcceleration(),
                                 config.IsReversed());

  // control interval sample traj
  const auto states = traj.States();

  size_t wptCnt = controlIntervalCounts.size() + 1;
  size_t sampTot = GetIdx(controlIntervalCounts, wptCnt, 0);

  Solution initialGuess{};
  initialGuess.x.reserve(sampTot);
  initialGuess.y.reserve(sampTot);
  initialGuess.theta.reserve(sampTot);
  initialGuess.dt.reserve(sampTot);

  size_t prevStateIdx = 0;
  size_t pointsPerSplineIdx = 0;

  const auto& firstPoint = initialGuessPoints.front().front();
  initialGuess.x.push_back(firstPoint.x);
  initialGuess.y.push_back(firstPoint.y);
  initialGuess.theta.push_back(firstPoint.heading);
  initialGuess.dt.push_back(0.0);

  for (size_t sgmtIdx = 1; sgmtIdx < initialGuessPoints.size(); ++sgmtIdx) {
    const auto& guessPointsForSgmt = initialGuessPoints.at(sgmtIdx);
    const size_t samplesForSgmt = controlIntervalCounts.at(sgmtIdx - 1);
    const size_t splinesInSgmt = guessPointsForSgmt.size();
    size_t samplesForSpline = samplesForSgmt / splinesInSgmt;

    size_t totalPointsInSgmt = 0;
    for (size_t i = 0; i < splinesInSgmt; ++i) {
      totalPointsInSgmt += pointsPerSpline.at(pointsPerSplineIdx) - 1;
      ++pointsPerSplineIdx;
    }
    const size_t endSgmtStateIdx = prevStateIdx + totalPointsInSgmt;
    const auto wholeSgmtDt =
        states.at(endSgmtStateIdx).t - states.at(prevStateIdx).t;
    const auto dt = wholeSgmtDt / static_cast<double>(samplesForSgmt);
    std::printf("dt for sgmt%zd with %zd splines: %.5f\n", sgmtIdx,
                splinesInSgmt, dt.value());

    for (size_t splineSgmtIdx = 0; splineSgmtIdx < splinesInSgmt;
         ++splineSgmtIdx) {
      double prevSamplesForSpline = static_cast<double>(samplesForSpline);
      if (splineSgmtIdx == splinesInSgmt - 1) {
        samplesForSpline += (samplesForSgmt % splinesInSgmt);
      }

      for (size_t sampleIdx = 1; sampleIdx <= samplesForSpline; ++sampleIdx) {
        auto t =
            states.at(prevStateIdx).t +
            (static_cast<double>(splineSgmtIdx) * prevSamplesForSpline + sampleIdx) * dt;
        const auto point = traj.Sample(t);
        initialGuess.x.push_back(point.pose.X().value());
        initialGuess.y.push_back(point.pose.Y().value());
        initialGuess.theta.push_back(point.pose.Rotation().Radians().value());
        initialGuess.dt.push_back(dt.value());
      }
    }
    prevStateIdx = endSgmtStateIdx;
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

  double totalTime = 0.0;
  std::printf("init solution: [\n");
  for (size_t i = 0; i < initialGuess.x.size(); ++i) {
    std::printf("[timestamp: %.3f, x: %.3f, y: %.3f, theta: %.3f]\n", totalTime,
                initialGuess.x.at(i), initialGuess.y.at(i),
                initialGuess.theta.at(i));
    totalTime += initialGuess.dt.at(i);
  }
  std::printf("]\n");

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
