#pragma once

#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Bumpers.h"
#include "obstacle/Obstacle.h"
#include "path/InitialGuessPoint.h"
#include "path/Path.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"

namespace trajopt {

class SwervePathBuilder {
 public:
  const SwervePath& GetPath() const;

  void SetDrivetrain(SwerveDrivetrain drivetrain);

  void PoseWpt(size_t idx, double x, double y, double heading);
  void TranslationWpt(size_t idx, double x, double y, double headingGuess = 0.0);

  void NewWpts(size_t finalIndex);

  void AddInitialGuessPoint(size_t fromIdx, double x, double y, double heading);

  void WptVelocityDirection(size_t idx, double angle);
  void WptVelocityMagnitude(size_t idx, double v);
  void WptZeroVelocity(size_t idx);
  void WptVelocityPolar(size_t idx, double vr, double vtheta);
  void WptZeroAngularVelocity(size_t idx);

  void SgmtVelocityDirection(size_t fromIdx, size_t toIdx, double angle, bool includeWpts = true);
  void SgmtVelocityMagnitude(size_t fromIdx, size_t toIdx, double v, bool includeWpts = true);
  void SgmtZeroAngularVelocity(size_t fromIdx, size_t toIdx, bool includeWpts = true);

  void WptConstraint(size_t idx, const HolonomicConstraint& constraint);
  void SgmtConstraint(size_t fromIdx, size_t toIdx, const HolonomicConstraint& constraint, bool includeWpts = true);

  void StartZeroVelocity();

  void AddBumpers(Bumpers&& newBumpers);
  void WptObstacle(size_t idx, const Obstacle& obstacle);
  void SgmtObstacle(size_t fromIdx, size_t toIdx, const Obstacle& obstacle, bool includeWpts = true);

  void ControlIntervalCounts(std::vector<size_t>&& counts);
  const std::vector<size_t>& GetControlIntervalCounts() const;

  Solution CalculateInitialGuess() const;

 private:
  SwervePath path;

  std::vector<Bumpers> bumpers;

  std::vector<std::vector<InitialGuessPoint>> initialGuessPoints;
  std::vector<size_t> controlIntervalCounts;

  static std::vector<HolonomicConstraint> GetConstraintsForObstacle(const Bumpers& bumpers, const Obstacle& obstacle);
};
}