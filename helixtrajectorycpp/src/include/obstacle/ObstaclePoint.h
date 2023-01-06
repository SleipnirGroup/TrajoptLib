// Copyright (c) TrajoptLib contributors

#pragma once

#include <iostream>

namespace helixtrajectory {

/**
 * @brief A point of an obstacle, usually representing a vertex of a polygon.
 *
 * @author Justin Babilino
 */
class ObstaclePoint {
 public:
  /**
   * @brief the x-coordinate of obstacle point
   */
  double x;
  /**
   * @brief the y-coordinate of obstacle point
   */
  double y;

  /**
   * @brief Construct a new Obstacle Point object with its position
   *
   * @param x x-coordinate
   * @param y y-coordinate
   */
  ObstaclePoint(double x, double y);

  /**
   * @brief Append a string representation of an obstacle point to an output
   * stream. A string representation of an obstacle point is a json object with
   * an "x" numerical field and a "y" numerical field.
   *
   * @param stream the stream to append the string representation to
   * @param point the obstacle point
   * @return a reference to the given stream
   */
  friend std::ostream& operator<<(std::ostream& stream,
                                  const ObstaclePoint& point);
};
}  // namespace helixtrajectory
