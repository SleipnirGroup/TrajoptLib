// // Copyright (c) TrajoptLib contributors

// #pragma once

// #include <vector>

// #include "constraint/Constraint.h"
// #include "obstacle/Obstacle.h"
// #include "path/Path.h"
// #include "set/IntervalSet1d.h"
// #include "set/Set2d.h"
// #include "solution/Solution.h"

// namespace trajopt {

// /**
//  * @brief This class is the superclass for all trajectory generators. It
//  * contains the common functionality of all optimizers: waypoint position
//  * constraints and obstacle avoidance.
//  */
// template <typename Opti>
// class TrajectoryOptimizationProblem {
//  protected:

//   /// The control interval counts of every segment (does not start with 0)
//   const std::vector<size_t> ctrlIntCnts;

//   /**
//    * @brief the number of waypoints in the path
//    */
//   const size_t wpCnt;
//   /**
//    * @brief the number of trajectory segments in the trajectory
//    */
//   const size_t segCnt;  // TODO: check if this can be removed
//   /**
//    * @brief the total number of control intervals in the trajectory
//    */
//   const size_t ctrlIntTot;
//   /**
//    * @brief the total number of sample points in the trajectory
//    * (controlIntervalTotal + 1)
//    */
//   const size_t sampTot;

//   /**
//    * @brief the optimizer
//    */
//   Opti opti;

//   /**
//    * @brief an abstract expression type representing a scalar expression
//    */
//   using Expression =
//       typename Opti::Expression;  // TODO: make this a template-specification
//                                   // pattern rather than typedefs

//   /**
//    * @brief The 1 x (controlIntervalTotal) vector of the time differentials
//    * between sample points. The nth entry in this vector is the duration of the
//    * nth interval in this trajectory.
//    */
//   std::vector<Expression> dt;

//   /**
//    * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's
//    * x-coordinate per trajectory sample point
//    */
//   std::vector<Expression> x;
//   /**
//    * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's
//    * y-coordinate per trajectory sample point
//    */
//   std::vector<Expression> y;
//   /**
//    * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's heading per
//    * trajectory sample point
//    */
//   std::vector<Expression> theta;

//   /**
//    * @brief Get the index of an item in a decision variable array, given the
//    * waypoint and sample indices and whether this array includes an entry
//    * for the initial sample point ("dt" does not, "x" does).
//    * 
//    * @param ctrlIntCnts the control interval counts of each segment, in order
//    * @param wpIdx the waypoint index (1 + segment index)
//    * @param sampIdx the sample index within the segment
//    * @param hasInitialSamp whether this decision variable includes a value for
//    * the initial sample
//    * @return the index in the array
//    */
//   static size_t GetIdx(const std::vector<size_t>& ctrlIntCnts,
//                        size_t wpIdx, size_t sampIdx = 0,
//                        bool hasInitialSamp = true);

//   /**
//    * @brief Construct a new CasADi Trajectory Optimization Problem from a
//    * drivetrain, path.
//    *
//    * @param drivetrain the drivetrain
//    * @param path the path
//    */
//   explicit TrajectoryOptimizationProblem(std::vector<size_t>&& ctrlIntCnts);

//   static void ApplyIntervalSet1dConstraint(Opti& opti, const Expression& scalar,
//                                            const IntervalSet1d& set1d);
//   static void ApplySet2dConstraint(Opti& opti, const Expression& vectorX,
//                                    const Expression& vectorY,
//                                    const Set2d& set2d);

//   static std::vector<double> SolutionValue(
//       const Opti& opti, const std::vector<Expression>& rowVector);
//   static std::vector<std::vector<double>> SolutionValue(
//       const Opti& opti, const std::vector<std::vector<Expression>>& matrix);

//   static void ApplyConstraint(Opti& opti, const Expression& x,
//                               const Expression& y, const Expression& theta,
//                               const Constraint& constraint);

//  private:
//   struct RobotPoint {
//     Expression x;
//     Expression y;
//   };

//   /**
//    * @brief Get an expression for the position of a bumper corner relative
//    * to the field coordinate system, given the robot's x-coordinate,
//    * y-coordinate, and heading. The first row of the resulting matrix contains
//    * the x-coordinate, and the second row contains the y-coordinate.
//    *
//    * @param x the instantaneous heading of the robot (scalar)
//    * @param y the instantaneous heading of the robot (scalar)
//    * @param theta the instantaneous heading of the robot (scalar)
//    * @param bumperCorner the bumper corner to find the position for
//    * @return the bumper corner 2 x 1 position vector
//    */
//   static const RobotPoint SolveRobotPointPosition(
//       const Expression& x, const Expression& y, const Expression& theta,
//       double robotPointX, double robotPointY);

//   template<typename Solution>
//   static void ApplyInitialGuess(Opti& opti, const Solution& solution,
//                                  std::vector<Expression>& x,
//                                  std::vector<Expression>& y,
//                                  std::vector<Expression>& theta);

//  public:
//   /**
//    * @brief Destroy the Trajectory Optimization Problem object
//    */
//   virtual ~TrajectoryOptimizationProblem() = default;
// };
// }  // namespace trajopt

// #include "optimization/TrajectoryOptimizationProblem.inc"
