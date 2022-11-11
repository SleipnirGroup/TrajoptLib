#include "DebugOptions.h"

#include "optimization/HolonomicTrajectoryOptimizationProblem.h"

#include <iostream>
#include <vector>

#include "optimization/CasADiOpti.h"
#include "drivetrain/HolonomicDrivetrain.h"
#include "path/HolonomicPath.h"
#include "trajectory/Trajectory.h"
#include "trajectory/TrajectorySample.h"
#include "path/HolonomicWaypoint.h"
#include "obstacle/Obstacle.h"
#include "TrajectoryGenerationException.h"

namespace helixtrajectory {

template<typename Opti>
HolonomicTrajectoryOptimizationProblem<Opti>::HolonomicTrajectoryOptimizationProblem(
        const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath)
        : TrajectoryOptimizationProblem<Opti>(holonomicDrivetrain, holonomicPath),
        holonomicDrivetrain(holonomicDrivetrain), holonomicPath(holonomicPath),
        vx(), vy(), omega(),
        ax(), ay(), alpha(),
        vxSegments(), vySegments(), omegaSegments(),
        axSegments(), aySegments(), alphaSegments() {

    vx.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
    vy.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
    omega.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
    ax.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
    ay.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
    alpha.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);

    for (size_t sampleIndex = 0; sampleIndex < TrajectoryOptimizationProblem<Opti>::sampleTotal; sampleIndex++) {
        // std::cout << "Setting up holonomic: " << std::endl;
        vx.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
        vy.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
        omega.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    }
    // std::cout << "Created new velocity variables" << std::endl;
    for (size_t intervalIndex = 0; intervalIndex < TrajectoryOptimizationProblem<Opti>::controlIntervalTotal; intervalIndex++) {
        ax.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
        ay.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
        alpha.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
    }
    // std::cout << "Created new acceleration variables" << std::endl;

    vxSegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
    vySegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
    omegaSegments.reserve(TrajectoryOptimizationProblem<Opti>::waypointCount);
    axSegments.reserve(TrajectoryOptimizationProblem<Opti>::trajectorySegmentCount);
    aySegments.reserve(TrajectoryOptimizationProblem<Opti>::trajectorySegmentCount);
    alphaSegments.reserve(TrajectoryOptimizationProblem<Opti>::trajectorySegmentCount);

    vxSegments.push_back({vx[0]});
    vySegments.push_back({vy[0]});
    omegaSegments.push_back({omega[0]});

    size_t sampleIndex = 1;
    for (size_t waypointIndex = 1; waypointIndex < TrajectoryOptimizationProblem<Opti>::waypointCount; waypointIndex++) {
        size_t controlIntervalCount = HolonomicTrajectoryOptimizationProblem::holonomicPath
                .holonomicWaypoints[waypointIndex].controlIntervalCount;
        std::vector<Expression> vxSegment; // TODO: this method of copyig vectors is probably inefficient
        std::vector<Expression> vySegment; // this could be fixed by moving the segments into the other vector
        std::vector<Expression> omegaSegment; // or by just using them directly in the vxSegments for example.
        std::vector<Expression> axSegment;
        std::vector<Expression> aySegment;
        std::vector<Expression> alphaSegment;
        vxSegment.reserve(controlIntervalCount);
        vySegment.reserve(controlIntervalCount);
        omegaSegment.reserve(controlIntervalCount);
        axSegment.reserve(controlIntervalCount);
        aySegment.reserve(controlIntervalCount);
        alphaSegment.reserve(controlIntervalCount);
        for (size_t segmentSampleIndex = 0; segmentSampleIndex < controlIntervalCount; segmentSampleIndex++) {
            vxSegment.push_back(vx[sampleIndex + segmentSampleIndex]);
            vySegment.push_back(vy[sampleIndex + segmentSampleIndex]);
            omegaSegment.push_back(omega[sampleIndex + segmentSampleIndex]);
            axSegment.push_back(ax[sampleIndex - 1 + segmentSampleIndex]);
            aySegment.push_back(ay[sampleIndex - 1 + segmentSampleIndex]);
            alphaSegment.push_back(alpha[sampleIndex - 1 + segmentSampleIndex]);
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
            TrajectoryOptimizationProblem<Opti>::theta,
            vx,
            vy,
            omega,
            ax,
            ay,
            alpha);

#ifdef DEBUG_OUTPUT
    std::cout << "Applied Holonomic Kinematics Constraints" << std::endl;
#endif

    ApplyHolonomicPathConstraints(
            TrajectoryOptimizationProblem<Opti>::opti,
            vxSegments,
            vySegments,
            omegaSegments,
            axSegments,
            aySegments,
            alphaSegments,
            HolonomicTrajectoryOptimizationProblem::holonomicPath);
#ifdef DEBUG_OUTPUT
    std::cout << "Applied Holonomic Path Constraints" << std::endl;
#endif
}

template<typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyKinematicsConstraints(Opti& opti,
        const std::vector<Expression>& dt,
        const std::vector<Expression>& x, const std::vector<Expression>& y, const std::vector<Expression>& theta,
        const std::vector<Expression>& vx, const std::vector<Expression>& vy, const std::vector<Expression>& omega,
        const std::vector<Expression>& ax, const std::vector<Expression>& ay, const std::vector<Expression>& alpha) {
    size_t sampleTotal = x.size();
    for (size_t sampleIndex = 1; sampleIndex < sampleTotal; sampleIndex++) {
        Expression sampleDT = dt[sampleIndex - 1];
        opti.SubjectTo(x[sampleIndex - 1] + vx[sampleIndex] * sampleDT == x[sampleIndex]);
        opti.SubjectTo(y[sampleIndex - 1] + vy[sampleIndex] * sampleDT == y[sampleIndex]);
        opti.SubjectTo(theta[sampleIndex - 1] + omega[sampleIndex] * sampleDT == theta[sampleIndex]);
        opti.SubjectTo(vx[sampleIndex - 1] + ax[sampleIndex - 1] * sampleDT == vx[sampleIndex]);
        opti.SubjectTo(vy[sampleIndex - 1] + ay[sampleIndex - 1] * sampleDT == vy[sampleIndex]);
        opti.SubjectTo(omega[sampleIndex - 1] + alpha[sampleIndex - 1] * sampleDT == omega[sampleIndex]);
    }
}

template<typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyHolonomicConstraint(Opti& opti,
        const Expression& vx, const Expression& vy, const Expression& omega,
        const Expression& ax, const Expression& ay, const Expression& alpha,
        const HolonomicConstraint& holonomicConstraint) {
    
    if (std::holds_alternative<VelocityHolonomicConstraint>(holonomicConstraint)) {
        const auto& velocityHolonomicConstraint = std::get<VelocityHolonomicConstraint>(holonomicConstraint);
        ApplySet2dConstraint(opti, vx, vy, velocityHolonomicConstraint.velocityBound);
    } else if (std::holds_alternative<AngularVelocityConstraint>(holonomicConstraint)) {
        const auto& angularVelocityConstraint = std::get<AngularVelocityConstraint>(holonomicConstraint);
        ApplyIntervalSet1dConstraint(opti, omega, angularVelocityConstraint.angularVelocityBound);
    }
}

template<typename Opti>
void ApplyHolonomicConstraints(Opti& opti,
        const typename Opti::Expression& vx,
        const typename Opti::Expression& vy,
        const typename Opti::Expression& omega,
        const typename Opti::Expression& ax,
        const typename Opti::Expression& ay,
        const typename Opti::Expression& alpha,
        const std::vector<HolonomicConstraint>& constraints) {
    for (const auto& constraint : constraints) {
        HolonomicTrajectoryOptimizationProblem<Opti>::ApplyHolonomicConstraint(opti, vx, vy, omega, ax, ay, alpha, constraint);
    }
}

template<typename Opti>
void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyHolonomicPathConstraints(Opti& opti,
        const std::vector<std::vector<Expression>>& vxSegments,
        const std::vector<std::vector<Expression>>& vySegments,
        const std::vector<std::vector<Expression>>& omegaSegments,
        const std::vector<std::vector<Expression>>& axSegments,
        const std::vector<std::vector<Expression>>& aySegments,
        const std::vector<std::vector<Expression>>& alphaSegments,
        const HolonomicPath& holonomicPath) {
    
    for (size_t waypointIndex = 0; waypointIndex < holonomicPath.holonomicWaypoints.size(); waypointIndex++) {
        const auto& holonomicWaypoint = holonomicPath.holonomicWaypoints[waypointIndex];
        size_t segmentSampleCount = holonomicWaypoint.controlIntervalCount;
        size_t waypointSampleIndex = segmentSampleCount - 1;
        for (size_t segmentSampleIndex = 0; segmentSampleIndex < segmentSampleCount - 1; segmentSampleIndex++) {
            ApplyHolonomicConstraints(opti,
                    vxSegments[waypointIndex][segmentSampleIndex],
                    vySegments[waypointIndex][segmentSampleIndex],
                    omegaSegments[waypointIndex][segmentSampleIndex],
                    axSegments[waypointIndex][segmentSampleIndex],
                    aySegments[waypointIndex][segmentSampleIndex],
                    alphaSegments[waypointIndex][segmentSampleIndex],
                    holonomicPath.globalHolonomicConstraints);
            ApplyHolonomicConstraints(opti,
                    vxSegments[waypointIndex][segmentSampleIndex],
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

template class HolonomicTrajectoryOptimizationProblem<CasADiOpti>;
}