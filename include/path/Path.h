// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "constraint/Constraint.h"
#include "obstacle/Obstacle.h"
#include "path/Waypoint.h"

namespace trajopt {

/**
 * @brief This class represents a sequence of waypoints on a 2D field. These
 * waypoints must be reached during the course of a trajectory.
 */
class TRAJOPT_DLLEXPORT Path {
 public:
  /**
   * @brief the boundaries of the robot's bumpers, represented as an Obstacle.
   */
  Obstacle bumpers;
  /**
   * @brief the constraint on position to be applied to all samples in
   * the trajectory.
   */
  std::vector<Constraint> globalConstraints;

  /**
   * @brief Get the number of waypoints that make up this path.
   *
   * @return the length of this path
   */
  virtual size_t Length() const noexcept = 0;
  /**
   * @brief Get the waypoint at the specified index.
   *
   * @param index the index
   * @return a reference to the waypoint
   */
  virtual Waypoint& GetWaypoint(size_t index) = 0;
  /**
   * @brief Get the waypoint at the specified index.
   *
   * @param index the index
   * @return a const reference to the waypoint
   */
  virtual const Waypoint& GetWaypoint(size_t index) const = 0;
  /**
   * @brief Get the number of control intervals of the entire path.
   *
   * @return the total number of control intervals in the path
   */
  size_t ControlIntervalTotal() const;

  /**
   * @brief Checks if this path is valid. A path is valid if it contains
   * no waypoints or the first waypoint has zero control intervals and
   * each waypoint in it is valid.
   *
   * @return true if this path is valid
   */
  bool IsValid() const noexcept;

  /**
   * @brief Destroy the Path object
   */
  virtual ~Path() = default;

 protected:
  Path(Obstacle bumpers, std::vector<Constraint> globalConstraints);
};
}  // namespace trajopt
