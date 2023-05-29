// #pragma once

// #include "path/Path.h"
// #include "solution/DifferentialSolution.h"

// namespace trajopt {

// template<typename Opti>
// class DifferentialTrajectoryOptimizationProblem {
//  public:
//   using Expression = typename Opti::Expression;

//   DifferentialSolution Generate();

//  private:
//   const DifferentialPath path;

//   /// number of waypoints in the path
//   const size_t wptCnt;
//   ///the number of trajectory segments in the trajectory
//   const size_t sgmtCnt;
//   /// the total number of control intervals in the trajectory
//   const size_t ctrlIntTot;
//   /// the total number of sample points in the trajectory (controlIntervalTotal + 1)
//   const size_t sampTot;

//   /// State Variables (x) -- size = sampTot
//   std::vector<Expression> x;
//   std::vector<Expression> y;
//   std::vector<Expression> theta;
//   std::vector<Expression> vl;
//   std::vector<Expression> vr;

//   /// Input Variables (u) -- size = sampTot
//   std::vector<std::vector<Expression>> Vl;
//   std::vector<std::vector<Expression>> Vr;

//   /// Time Variables -- size = sgmtCnt
//   std::vector<Expression> dt;

//   Opti opti;

//   static void ApplyDynamics(Opti& opti);

//   Expression SolveT(const Expression& dt);

//  public:
//   /**
//    * @brief Construct a new CasADi Swerve Trajectory Optimization Problem
//    * with a swerve drivetrain and holonomic path.
//    *
//    * @param swerveDrivetrain the swerve drivetrain
//    * @param holonomicPath the holonomic path
//    */
//   explicit DifferentialTrajectoryOptimizationProblem(const SwervePath& swervePath);

// };
// }

// #include "optimization/DifferentialTrajectoryOptimizationProblem.inc"