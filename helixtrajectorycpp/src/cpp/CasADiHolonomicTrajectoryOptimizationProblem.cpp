#include "CasADiHolonomicTrajectoryOptimizationProblem.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicDrivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "HolonomicTrajectorySample.h"
#include "HolonomicTrajectorySegment.h"
#include "HolonomicWaypoint.h"
#include "Obstacle.h"
#include "TrajectoryGenerationException.h"

namespace helixtrajectory {

    CasADiHolonomicTrajectoryOptimizationProblem::CasADiHolonomicTrajectoryOptimizationProblem(
            const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath)
            : CasADiTrajectoryOptimizationProblem(holonomicDrivetrain, holonomicPath),
            holonomicDrivetrain(holonomicDrivetrain), holonomicPath(holonomicPath),
            V(opti.variable(3, controlIntervalTotal + 1)), vx(V(0, ALL)), vy(V(1, ALL)), omega(V(2, ALL)),
            U(opti.variable(3, controlIntervalTotal)), ax(U(0, ALL)), ay(U(1, ALL)), alpha(U(2, ALL)) {

        VSegments.reserve(waypointCount);
        vxSegments.reserve(waypointCount);
        vySegments.reserve(waypointCount);
        omegaSegments.reserve(waypointCount);
        USegments.reserve(trajectorySegmentCount);

        VSegments.push_back(V(ALL, 0));
        vxSegments.push_back(vx(ALL, 0));
        vySegments.push_back(vy(ALL, 0));
        omegaSegments.push_back(omega(ALL, 0));

        size_t sampleIndex = 1;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t controlIntervalCount = CasADiHolonomicTrajectoryOptimizationProblem::holonomicPath
                    .holonomicWaypoints[waypointIndex].controlIntervalCount;

            casadi::Slice VSlice((int) sampleIndex, (int) (sampleIndex + controlIntervalCount));
            casadi::Slice USlice((int) (sampleIndex - 1), (int) (sampleIndex - 1 + controlIntervalCount));

            VSegments.push_back(V(ALL, VSlice));
            vxSegments.push_back(vx(ALL, VSlice));
            vySegments.push_back(vy(ALL, VSlice));
            omegaSegments.push_back(omega(ALL, VSlice));
            USegments.push_back(U(ALL, USlice));

            sampleIndex += controlIntervalCount;
        }

        ApplyKinematicsConstraints(opti, dt, X, V, U);
        std::cout << "Applied Holonomic Kinematics Constraints" << std::endl;

        std::cout << "Applied Swerve Dynamics Constraints" << std::endl;
        
        ApplyHolonomicWaypointConstraints(opti, vxSegments, vySegments, omegaSegments,
                CasADiHolonomicTrajectoryOptimizationProblem::holonomicPath);
        std::cout << "Applied Holonomic Path Constraints" << std::endl;
    }

    HolonomicTrajectory CasADiHolonomicTrajectoryOptimizationProblem::Generate() {
        // I don't try-catch this next line since it should always work.
        // I'm assuming the dynamic lib is on the path and casadi can find it.
        opti.solver("ipopt");
        std::cout << "Located IPOPT Plugin" << std::endl;
        try {
            auto solution = opti.solve();
            std::cout << "Solution Found" << std::endl;
            return ConstructTrajectory(solution, dtSegments, xSegments, ySegments, thetaSegments,
                vxSegments, vySegments, omegaSegments);
        } catch (...) {
            throw TrajectoryGenerationException("Error optimizing trajectory");
        }
    }

    void CasADiHolonomicTrajectoryOptimizationProblem::ApplyKinematicsConstraints(casadi::Opti& opti,
            const casadi::MX& dt, const casadi::MX& X, const casadi::MX& V, const casadi::MX& U) {
        size_t sampleTotal = X.columns();
        for (size_t sampleIndex = 1; sampleIndex < sampleTotal; sampleIndex++) {
            casadi::MX sampleDT = dt(sampleIndex - 1);
            opti.subject_to(X(ALL, sampleIndex - 1) + V(ALL, sampleIndex) * sampleDT == X(ALL, sampleIndex));
            opti.subject_to(V(ALL, sampleIndex - 1) + U(ALL, sampleIndex - 1) * sampleDT == V(ALL, sampleIndex));
        }
    }

    HolonomicTrajectory CasADiHolonomicTrajectoryOptimizationProblem::ConstructTrajectory(const casadi::OptiSol& solution,
            const std::vector<casadi::MX>& dtSegments,
            const std::vector<casadi::MX>& xSegments, const std::vector<casadi::MX>& ySegments,
            const std::vector<casadi::MX>& thetaSegments, const std::vector<casadi::MX>& vxSegments,
            const std::vector<casadi::MX>& vySegments, const std::vector<casadi::MX>& omegaSegments) {

        std::vector<HolonomicTrajectorySegment> segments;
        segments.reserve(xSegments.size());

        segments.push_back(HolonomicTrajectorySegment({HolonomicTrajectorySample(
                0.0,
                static_cast<double>(solution.value(xSegments[0](0))),
                static_cast<double>(solution.value(ySegments[0](0))),
                static_cast<double>(solution.value(thetaSegments[0](0))),
                static_cast<double>(solution.value(vxSegments[0](0))),
                static_cast<double>(solution.value(vySegments[0](0))),
                static_cast<double>(solution.value(omegaSegments[0](0))))}));

        for (size_t xSegmentIndex = 1; xSegmentIndex < xSegments.size(); xSegmentIndex++) {
            size_t segmentSampleCount = xSegments[xSegmentIndex].columns();
            std::vector<HolonomicTrajectorySample> samples;
            samples.reserve(segmentSampleCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < segmentSampleCount; segmentSampleIndex++) {
                samples.push_back(HolonomicTrajectorySample(
                        static_cast<double>(solution.value(dtSegments[xSegmentIndex - 1](segmentSampleIndex))),
                        static_cast<double>(solution.value(xSegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(ySegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(thetaSegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(vxSegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(vySegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(omegaSegments[xSegmentIndex](segmentSampleIndex)))));
            }
            segments.push_back(HolonomicTrajectorySegment(samples));
        }

        return HolonomicTrajectory(segments);
    }

    void CasADiHolonomicTrajectoryOptimizationProblem::ApplyHolonomicWaypointConstraints(casadi::Opti& opti,
            const std::vector<casadi::MX>& vxSegments, const std::vector<casadi::MX>& vySegments,
            const std::vector<casadi::MX>& omegaSegments, const HolonomicPath& holonomicPath) {
        size_t waypointCount = vxSegments.size();
        for (size_t waypointIndex = 0; waypointIndex < waypointCount; waypointIndex++) {
            const HolonomicWaypoint& waypoint = holonomicPath.holonomicWaypoints[waypointIndex];
            if (waypoint.velocityMagnitudeConstrained) {
                if (waypoint.velocityXConstrained) {
                    opti.subject_to(vxSegments[waypointIndex](-1) == waypoint.velocityX);
                }
                if (waypoint.velocityYConstrained) {
                    opti.subject_to(vySegments[waypointIndex](-1) == waypoint.velocityY);
                }
                if (!waypoint.velocityXConstrained && !waypoint.velocityYConstrained) {
                    if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.subject_to(vxSegments[waypointIndex](-1) == 0.0);
                        opti.subject_to(vySegments[waypointIndex](-1) == 0.0);
                    } else {
                        double magnitudeSquared = waypoint.velocityX * waypoint.velocityX + waypoint.velocityY * waypoint.velocityY;
                        opti.subject_to(vxSegments[waypointIndex](-1) * vxSegments[waypointIndex](-1)
                                + vySegments[waypointIndex](-1) * vySegments[waypointIndex](-1) == magnitudeSquared);
                    }
                }
            } else if (waypoint.velocityXConstrained && waypoint.velocityYConstrained) {
                if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.subject_to(vxSegments[waypointIndex](-1) == 0.0);
                        opti.subject_to(vySegments[waypointIndex](-1) == 0.0);
                } else {
                    casadi::MX scalarMultiplier = opti.variable();
                    opti.subject_to(vxSegments[waypointIndex](-1) == scalarMultiplier * waypoint.velocityX);
                    opti.subject_to(vySegments[waypointIndex](-1) == scalarMultiplier * waypoint.velocityY);
                }
            }
            if (waypoint.angularVelocityConstrained) {
                opti.subject_to(omegaSegments[waypointIndex](-1) == waypoint.angularVelocity);
            }
        }
    }
}