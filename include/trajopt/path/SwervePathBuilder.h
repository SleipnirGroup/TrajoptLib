// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "trajopt/drivetrain/SwerveDrivetrain.h"
#include "trajopt/obstacle/Bumpers.h"
#include "trajopt/obstacle/Obstacle.h"
#include "trajopt/path/InitialGuessPoint.h"
#include "trajopt/path/Path.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/Set2d.h"

namespace trajopt {

/**
 * Builds a swerve path using information about how the robot
 * must travel through a series of waypoints. This path can be converted
 * to a trajectory using OptimalTrajectoryGenerator.
 */
class TRAJOPT_DLLEXPORT SwervePathBuilder {
 public:
  /**
   * Cancel all currently generating SwervePathBuilders.
   */
  void CancelAll();

  /**
   * Get the SwervePath being constructed
   *
   * @return the path
   */
  const SwervePath& GetPath() const;

  /**
   * Set the Drivetrain object
   *
   * @param drivetrain the new drivetrain
   */
  void SetDrivetrain(SwerveDrivetrain drivetrain);

  /**
   * Create a pose waypoint constraint on the waypoint at the provided
   * index, and add an initial guess with the same pose This specifies that the
   * position and heading of the robot at the waypoint must be fixed at the
   * values provided.
   *
   * @param idx index of the pose waypoint
   * @param x the x
   * @param y the y
   * @param heading the heading
   */
  void PoseWpt(size_t idx, double x, double y, double heading);
  /**
   * Create a translation waypoint constraint on the waypoint at the
   * provided index, and add an initial guess point with the same translation.
   * This specifies that the position of the robot at the waypoint must be fixed
   * at the value provided.
   *
   * @param idx index of the pose waypoint
   * @param x the x
   * @param y the y
   * @param headingGuess optionally, an initial guess of the heading
   */
  void TranslationWpt(size_t idx, double x, double y,
                      double headingGuess = 0.0);

  /**
   * Provide a guess of the instantaneous pose of the robot at a waypoint.
   *
   * @param wptIdx the waypoint to apply the guess to
   * @param poseGuess the guess of the robot's pose
   */
  void WptInitialGuessPoint(size_t wptIdx, const InitialGuessPoint& poseGuess);

  /**
   * Add a sequence of initial guess points between two waypoints. The points
   * are inserted between the waypoints at fromIdx and fromIdx + 1. Linear
   * interpolation between the waypoint initial guess points and these segment
   * initial guess points is used as the initial guess of the robot's pose over
   * the trajectory.
   *
   * @param fromIdx index of the waypoint the initial guess point
   *                 comes immediately after
   * @param sgmtPoseGuess the sequence of initial guess points
   */
  void SgmtInitialGuessPoints(
      size_t fromIdx, const std::vector<InitialGuessPoint>& sgmtPoseGuess);

  /**
   * Specify the required direction of the velocity vector of the robot
   * at a waypoint.
   *
   * @param idx index of the waypoint
   * @param angle the polar angle of the required direction
   */
  void WptVelocityDirection(size_t idx, double angle);

  /**
   * Specify the required maximum magnitude of the velocity vector of the
   * robot at a waypoint.
   *
   * @param idx index of the waypoint
   * @param v the maximum velocity magnitude
   */
  void WptVelocityMagnitude(size_t idx, double v);

  /**
   * Specify the required velocity vector of the robot at a waypoint to
   * be zero.
   *
   * @param idx index of the waypoint
   */
  void WptZeroVelocity(size_t idx);

  /**
   * Specify the exact required velocity vector of the robot at a
   * waypoint.
   *
   * @param idx index of the waypoint
   * @param vr velocity vector magnitude
   * @param vtheta velocity vector polar angle
   */
  void WptVelocityPolar(size_t idx, double vr, double vtheta);
  /**
   * Specify the required angular velocity of the robot to be zero
   * at a waypoint
   *
   * @param idx index of the waypoint
   */
  void WptZeroAngularVelocity(size_t idx);

  /**
   * Specify the required direction of the velocity vector of the robot
   * for the continuum of robot state between two waypoints.
   *
   * @param fromIdx the waypoint at the beginning of the continuum
   * @param toIdx the waypoint at the end of the continuum
   * @param angle the polar angle of the required direction
   * @param includeWpts if using a discrete algorithm, false does not apply the
   * constraint at the instantaneous state at waypoints at indices fromIdx and
   * toIdx
   */
  void SgmtVelocityDirection(size_t fromIdx, size_t toIdx, double angle,
                             bool includeWpts = true);

  /**
   * Specify the required maximum magnitude of the velocity vector of the
   * robot for the continuum of robot state between two waypoints.
   *
   * @param fromIdx the waypoint at the beginning of the continuum
   * @param toIdx the waypoint at the end of the continuum
   * @param v the maximum velocity magnitude
   * @param includeWpts if using a discrete algorithm, false does not apply the
   * constraint at the instantaneous state at waypoints at indices fromIdx and
   * toIdx
   */
  void SgmtVelocityMagnitude(size_t fromIdx, size_t toIdx, double v,
                             bool includeWpts = true);

  /**
   * Specify the required angular velocity of the robot to be zero
   * for the continuum of robot state between two waypoints.
   *
   * @param fromIdx index of the waypoint at the beginning of the continuum
   * @param toIdx index of the waypoint at the end of the continuum
   * @param includeWpts if using a discrete algorithm, false does not apply the
   * constraint at the instantaneous state at waypoints at indices fromIdx and
   * toIdx
   */
  void SgmtZeroAngularVelocity(size_t fromIdx, size_t toIdx,
                               bool includeWpts = true);

  /**
   * Apply a custom holonomic constraint at a waypoint
   *
   * @param idx index of the waypoint
   * @param constraint the constraint to be applied
   */
  void WptConstraint(size_t idx, const HolonomicConstraint& constraint);

  /**
   * Apply a custom holonomic constraint to the continuum of state
   * between two waypoints.
   *
   * @param fromIdx index of the waypoint at the beginning of the continuum
   * @param toIdx index of the waypoint at the end of the continuum
   * @param constraint the custom constraint to be applied
   * @param includeWpts if using a discrete algorithm, false does not apply the
   * constraint at the waypoints at fromIdx and toIdx
   */
  void SgmtConstraint(size_t fromIdx, size_t toIdx,
                      const HolonomicConstraint& constraint,
                      bool includeWpts = true);

  /**
   * Add polygon or circle shaped bumpers to a list used when applying
   * obstacle constraints.
   *
   * @param newBumpers bumpers to add
   */
  void AddBumpers(Bumpers&& newBumpers);

  /**
   * Apply an obstacle constraint to a waypoint.
   *
   * @param idx index of the waypoint
   * @param obstacle the obstacle
   */
  void WptObstacle(size_t idx, const Obstacle& obstacle);

  /**
   * Apply an obstacle constraint to the continuum of state between two
   * waypoints.
   *
   * @param fromIdx index of the waypoint at the beginning of the continuum
   * @param toIdx index of the waypoint at the end of the continuum
   * @param obstacle the obstacle
   * @param includeWpts if using a discrete algorithm, false does not apply the
   * constraint at the instantaneous state at waypoints at indices fromIdx and
   * toIdx
   */
  void SgmtObstacle(size_t fromIdx, size_t toIdx, const Obstacle& obstacle,
                    bool includeWpts = true);

  /**
   * If using a discrete algorithm, specify the number of discrete
   * samples for every segment of the trajectory
   *
   * @param counts the sequence of control interval counts per segment, length
   * is number of waypoints - 1
   */
  void ControlIntervalCounts(std::vector<size_t>&& counts);

  /**
   * Get the Control Interval Counts object
   *
   * @return const std::vector<size_t>&
   */
  const std::vector<size_t>& GetControlIntervalCounts() const;

  /**
   * Calculate a discrete, linear initial guess of the x, y, and heading
   * of the robot that goes through each segment.
   *
   * @return the initial guess, as a solution
   */
  Solution CalculateInitialGuess() const;

 private:
  SwervePath path;

  std::vector<Bumpers> bumpers;

  std::vector<std::vector<InitialGuessPoint>> initialGuessPoints;
  std::vector<size_t> controlIntervalCounts;

  void NewWpts(size_t finalIndex);

  static std::vector<HolonomicConstraint> GetConstraintsForObstacle(
      const Bumpers& bumpers, const Obstacle& obstacle);
};
}  // namespace trajopt
