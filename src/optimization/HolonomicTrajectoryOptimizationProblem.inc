// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "DebugOptions.h"
#include "TrajectoryGenerationException.h"
#include "drivetrain/HolonomicDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "optimization/HolonomicTrajectoryOptimizationProblem.h"
#include "path/HolonomicPath.h"
#include "path/HolonomicWaypoint.h"
#include "solution/HolonomicSolution.h"

namespace trajopt {

template <typename Opti>
HolonomicTrajectoryOptimizationProblem<Opti>::
    HolonomicTrajectoryOptimizationProblem(
        const HolonomicDrivetrain& holonomicDrivetrain,
        const HolonomicPath& holonomicPath)
    : TrajectoryOptimizationProblem<Opti>(holonomicDrivetrain, holonomicPath),
      holonomicDrivetrain(holonomicDrivetrain),
      holonomicPath(holonomicPath),
      vx(),
      vy(),
      omega(),
      ax(),
      ay(),
      alpha(),
      vxSegments(),
      vySegments(),
      omegaSegments(),
      axSegments(),
      aySegments(),
      alphaSegments() {
  vx.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
  vy.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
  omega.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
  ax.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
  ay.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
  alpha.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);

  for (size_t sampleIndex = 0;
       sampleIndex < TrajectoryOptimizationProblem<Opti>::sampleTotal;
       sampleIndex++) {
    vx.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    vy.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    omega.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    ax.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    ay.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    alpha.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
  }

  vxSegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
  vySegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
  omegaSegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
  axSegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
  aySegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
  alphaSegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);

  vxSegments.push_back({vx[0]});
  vySegments.push_back({vy[0]});
  omegaSegments.push_back({omega[0]});
  axSegments.push_back({ax[0]});
  aySegments.push_back({ay[0]});
  alphaSegments.push_back({alpha[0]});

  size_t sampleIndex = 1;
  for (size_t waypointIndex = 1;
       waypointIndex < TrajectoryOptimizationProblem<Opti>::waypointCount;
       waypointIndex++) {
    size_t controlIntervalCount =
        HolonomicTrajectoryOptimizationProblem::holonomicPath
            .holonomicWaypoints[waypointIndex]
            .controlIntervalCount;
    std::vector<Expression> vxSegment;  // TODO: this method of copyig vectors
                                        // is probably inefficient
    std::vector<Expression> vySegment;  // this could be fixed by moving the
                                        // segments into the other vector
    std::vector<Expression> omegaSegment;  // or by just using them directly in
                                           // the vxSegments for example.
    std::vector<Expression> axSegment;
    std::vector<Expression> aySegment;
    std::vector<Expression> alphaSegment;
    vxSegment.reserve(controlIntervalCount);
    vySegment.reserve(controlIntervalCount);
    omegaSegment.reserve(controlIntervalCount);
    axSegment.reserve(controlIntervalCount);
    aySegment.reserve(controlIntervalCount);
    alphaSegment.reserve(controlIntervalCount);
    for (size_t segmentSampleIndex = 0;
         segmentSampleIndex < controlIntervalCount; segmentSampleIndex++) {
      vxSegment.push_back(vx[sampleIndex + segmentSampleIndex]);
      vySegment.push_back(vy[sampleIndex + segmentSampleIndex]);
      omegaSegment.push_back(omega[sampleIndex + segmentSampleIndex]);
      axSegment.push_back(ax[sampleIndex + segmentSampleIndex]);
      aySegment.push_back(ay[sampleIndex + segmentSampleIndex]);
      alphaSegment.push_back(alpha[sampleIndex + segmentSampleIndex]);
    }
    vxSegments.push_back(vxSegment);
    vySegments.push_back(vySegment);
    omegaSegments.push_back(omegaSegment);
    axSegments.push_back(axSegment);
    aySegments.push_back(aySegment);
    alphaSegments.push_back(alphaSegment);
    sampleIndex += controlIntervalCount;
  }

  ApplyKinematicsConstraints(TrajectoryOptimizationProblem<Opti>::opti,
                             TrajectoryOptimizationProblem<Opti>::dt,
                             TrajectoryOptimizationProblem<Opti>::x,
                             TrajectoryOptimizationProblem<Opti>::y,
                             TrajectoryOptimizationProblem<Opti>::theta, vx, vy,
                             omega, ax, ay, alpha);

  ApplyHolonomicPathConstraints(
      TrajectoryOptimizationProblem<Opti>::opti, vxSegments, vySegments,
      omegaSegments, axSegments, aySegments, alphaSegments,
      HolonomicTrajectoryOptimizationProblem::holonomicPath);
}

template <typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyKinematicsConstraints(
    Opti& opti, const std::vector<Expression>& dt,
    const std::vector<Expression>& x, const std::vector<Expression>& y,
    const std::vector<Expression>& theta, const std::vector<Expression>& vx,
    const std::vector<Expression>& vy, const std::vector<Expression>& omega,
    const std::vector<Expression>& ax, const std::vector<Expression>& ay,
    const std::vector<Expression>& alpha) {
  size_t sampleTotal = x.size();
  for (size_t sampleIndex = 1; sampleIndex < sampleTotal; sampleIndex++) {
    Expression sampleDT = dt[sampleIndex - 1];
    opti.SubjectTo(x[sampleIndex - 1] + vx[sampleIndex] * sampleDT ==
                   x[sampleIndex]);
    opti.SubjectTo(y[sampleIndex - 1] + vy[sampleIndex] * sampleDT ==
                   y[sampleIndex]);
    opti.SubjectTo(theta[sampleIndex - 1] + omega[sampleIndex] * sampleDT ==
                   theta[sampleIndex]);
    opti.SubjectTo(vx[sampleIndex - 1] + ax[sampleIndex] * sampleDT ==
                   vx[sampleIndex]);
    opti.SubjectTo(vy[sampleIndex - 1] + ay[sampleIndex] * sampleDT ==
                   vy[sampleIndex]);
    opti.SubjectTo(omega[sampleIndex - 1] + alpha[sampleIndex] * sampleDT ==
                   omega[sampleIndex]);
  }
}

template <typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyHolonomicConstraint(
    Opti& opti, const Expression& vx, const Expression& vy,
    const Expression& omega, const Expression& ax, const Expression& ay,
    const Expression& alpha, const HolonomicConstraint& holonomicConstraint) {
  if (std::holds_alternative<HolonomicVelocityConstraint>(holonomicConstraint)) {
    const auto& velocityHolonomicConstraint =
        std::get<HolonomicVelocityConstraint>(holonomicConstraint);
    TrajectoryOptimizationProblem<Opti>::ApplySet2dConstraint(
        opti, vx, vy, velocityHolonomicConstraint.velocityBound);
  } else if (std::holds_alternative<AngularVelocityConstraint>(
                 holonomicConstraint)) {
    const auto& angularVelocityConstraint =
        std::get<AngularVelocityConstraint>(holonomicConstraint);
    TrajectoryOptimizationProblem<Opti>::ApplyIntervalSet1dConstraint(
        opti, omega, angularVelocityConstraint.angularVelocityBound);
  }
}

template <typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyHolonomicConstraints(
    Opti& opti, const typename Opti::Expression& vx,
    const typename Opti::Expression& vy, const typename Opti::Expression& omega,
    const typename Opti::Expression& ax, const typename Opti::Expression& ay,
    const typename Opti::Expression& alpha,
    const std::vector<HolonomicConstraint>& constraints) {
  for (const auto& constraint : constraints) {
    ApplyHolonomicConstraint(opti, vx, vy, omega, ax, ay, alpha, constraint);
  }
}

template <typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::
    ApplyHolonomicPathConstraints(
        Opti& opti, const std::vector<std::vector<Expression>>& vxSegments,
        const std::vector<std::vector<Expression>>& vySegments,
        const std::vector<std::vector<Expression>>& omegaSegments,
        const std::vector<std::vector<Expression>>& axSegments,
        const std::vector<std::vector<Expression>>& aySegments,
        const std::vector<std::vector<Expression>>& alphaSegments,
        const HolonomicPath& holonomicPath) {
  ApplyHolonomicConstraints(opti, vxSegments[0][0], vySegments[0][0],
                            omegaSegments[0][0], axSegments[0][0],
                            aySegments[0][0], alphaSegments[0][0],
                            holonomicPath.globalHolonomicConstraints);
  ApplyHolonomicConstraints(
      opti, vxSegments[0][0], vySegments[0][0], omegaSegments[0][0],
      axSegments[0][0], aySegments[0][0], alphaSegments[0][0],
      holonomicPath.holonomicWaypoints[0].waypointHolonomicConstraints);

  for (size_t waypointIndex = 1;
       waypointIndex < holonomicPath.holonomicWaypoints.size();
       waypointIndex++) {
    const auto& holonomicWaypoint =
        holonomicPath.holonomicWaypoints[waypointIndex];
    size_t segmentSampleCount = holonomicWaypoint.controlIntervalCount;
    size_t waypointSampleIndex = segmentSampleCount - 1;
    for (size_t segmentSampleIndex = 0;
         segmentSampleIndex < segmentSampleCount - 1; segmentSampleIndex++) {
      ApplyHolonomicConstraints(
          opti, vxSegments[waypointIndex][segmentSampleIndex],
          vySegments[waypointIndex][segmentSampleIndex],
          omegaSegments[waypointIndex][segmentSampleIndex],
          axSegments[waypointIndex][segmentSampleIndex],
          aySegments[waypointIndex][segmentSampleIndex],
          alphaSegments[waypointIndex][segmentSampleIndex],
          holonomicPath.globalHolonomicConstraints);
      ApplyHolonomicConstraints(
          opti, vxSegments[waypointIndex][segmentSampleIndex],
          vySegments[waypointIndex][segmentSampleIndex],
          omegaSegments[waypointIndex][segmentSampleIndex],
          axSegments[waypointIndex][segmentSampleIndex],
          aySegments[waypointIndex][segmentSampleIndex],
          alphaSegments[waypointIndex][segmentSampleIndex],
          holonomicWaypoint.segmentHolonomicConstraints);
    }
    ApplyHolonomicConstraints(opti,
                              vxSegments[waypointIndex][waypointSampleIndex],
                              vySegments[waypointIndex][waypointSampleIndex],
                              omegaSegments[waypointIndex][waypointSampleIndex],
                              axSegments[waypointIndex][waypointSampleIndex],
                              aySegments[waypointIndex][waypointSampleIndex],
                              alphaSegments[waypointIndex][waypointSampleIndex],
                              holonomicPath.globalHolonomicConstraints);
    ApplyHolonomicConstraints(opti,
                              vxSegments[waypointIndex][waypointSampleIndex],
                              vySegments[waypointIndex][waypointSampleIndex],
                              omegaSegments[waypointIndex][waypointSampleIndex],
                              axSegments[waypointIndex][waypointSampleIndex],
                              aySegments[waypointIndex][waypointSampleIndex],
                              alphaSegments[waypointIndex][waypointSampleIndex],
                              holonomicWaypoint.waypointHolonomicConstraints);
  }
}

// template<typename Opti>
// HolonomicSolution
// HolonomicTrajectoryOptimizationProblem<Opti>::ConstructTrajectory(const Opti&
// opti,
//         const std::vector<Expression>& dt,
//         const std::vector<Expression>& x,
//         const std::vector<Expression>& y,
//         const std::vector<Expression>& theta,
//         const std::vector<Expression>& vx,
//         const std::vector<Expression>& vy,
//         const std::vector<Expression>& omega,
//         const std::vector<Expression>& ax,
//         const std::vector<Expression>& ay,
//         const std::vector<Expression>& alpha) {

//     HolonomicState initialState(
//             static_cast<double>(opti.SolutionValue(x[0])),
//             static_cast<double>(opti.SolutionValue(y[0])),
//             static_cast<double>(opti.SolutionValue(theta[0])),
//             static_cast<double>(opti.SolutionValue(vx[0])),
//             static_cast<double>(opti.SolutionValue(vy[0])),
//             static_cast<double>(opti.SolutionValue(omega[0])),
//             static_cast<double>(opti.SolutionValue(ax[0])),
//             static_cast<double>(opti.SolutionValue(ay[0])),
//             static_cast<double>(opti.SolutionValue(alpha[0])));

//     std::vector<HolonomicTrajectorySample> samples;
//     samples.reserve(x.size() - 1);

//     for (size_t sampleIndex = 1; sampleIndex < x.size(); sampleIndex++) {
//         // TODO: check if emplace_back works?
//         samples.push_back(HolonomicTrajectorySample(
//                 static_cast<double>(opti.SolutionValue(dt[sampleIndex - 1])),
//                 HolonomicState(
//                     static_cast<double>(opti.SolutionValue(x[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(y[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(theta[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(vx[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(vy[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(omega[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(ax[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(ay[sampleIndex])),
//                     static_cast<double>(opti.SolutionValue(alpha[sampleIndex]))
//                 )));
//     }

//     return HolonomicTrajectory(initialState, samples);
// }
}  // namespace trajopt
