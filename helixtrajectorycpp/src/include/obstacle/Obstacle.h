// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>
#include <vector>

#include "obstacle/ObstaclePoint.h"

namespace helixtrajectory {

/**
 * @brief Represents a physical obstacle that the robot must avoid by a certain
 * distance. Arbitrary polygons can be expressed with this class, and circle
 * obstacles can also be created by only using one point with a safety distance.
 * Obstacle points must be wound either clockwise or counterclockwise.
 *
 * @author Justin Babilino
 */
class Obstacle {
 public:
  /**
   * @brief minimum distance from the obstacle the robot must maintain
   */
  double safetyDistance;
  /**
   * @brief the list of points that make up this obstacle
   */
  std::vector<ObstaclePoint> points;

  /**
   * @brief Construct a new Obstacle object
   *
   * @param safetyDistance the list of points that make up this obstacle
   * @param applyToAllSegments whether to apply this obstacle to all trajectory
   * segments
   * @param waypoints minimum distance from the obstacle the robot must maintain
   */
  Obstacle(double safetyDistance, const std::vector<ObstaclePoint>& points);

  /**
   * @brief Append a string representation of an obstacle to an output stream.
   * A string representation of an obstacle is a json object with a
   * "safety_distance" numerical field and a "points" array field with the
   * ordered list of obstacle points.
   *
   * @param stream the stream to append the string representation to
   * @param sample the obstacle
   * @return a reference to the given stream
   */
  friend std::ostream& operator<<(std::ostream& stream,
                                  const Obstacle& obstacle);
};
}  // namespace helixtrajectory
