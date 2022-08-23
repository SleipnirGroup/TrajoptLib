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

        CasADiHolonomicTrajectoryOptimizationProblem::ApplyKinematicsConstraints(opti, XSegments, VSegments, USegments, trajectorySegmentDts);
        std::cout << "Applied Holonomic Kinematics Constraints" << std::endl;

        CasADiHolonomicTrajectoryOptimizationProblem::holonomicDrivetrain.ApplyDynamicsConstraints(
                opti, theta, vx, vy, omega, ax, ay, alpha, controlIntervalTotal);
        std::cout << "Applied Swerve Dynamics Constraints" << std::endl;
        
        ApplyHolonomicPathConstraints();
        std::cout << "Applied Holonomic Path Constraints" << std::endl;
    }

    HolonomicTrajectory CasADiHolonomicTrajectoryOptimizationProblem::Generate() {
        size_t controlIntervalTotal = path.ControlIntervalTotal();
        if (holonomicPath.Length() == 1) {
            return HolonomicTrajectory({HolonomicTrajectorySegment(0.0, {HolonomicTrajectorySample(
                    holonomicPath.holonomicWaypoints[0].x,
                    holonomicPath.holonomicWaypoints[0].y,
                    holonomicPath.holonomicWaypoints[0].heading,
                    holonomicPath.holonomicWaypoints[0].velocityX,
                    holonomicPath.holonomicWaypoints[0].velocityY,
                    holonomicPath.holonomicWaypoints[0].angularVelocity
            )})});
        }
        opti.solver("ipopt");
        std::cout << "Located IPOPT Plugin" << std::endl;
        auto solution = opti.solve();
        std::cout << "Solution Found" << std::endl;

        return ConstructTrajectory(solution, xSegments, ySegments, thetaSegments,
                vxSegments, vySegments, omegaSegments, trajectorySegmentDts);
    }

    void CasADiHolonomicTrajectoryOptimizationProblem::ApplyKinematicsConstraints(casadi::Opti& opti, const std::vector<casadi::MX>& XSegments,
            const std::vector<casadi::MX>& VSegments, const std::vector<casadi::MX>& USegments,
            const casadi::MX& segmentDts) {
        for (size_t XSegmentIndex = 1; XSegmentIndex < XSegments.size(); XSegmentIndex++) {
            size_t previousSegmentSampleCount = XSegments[XSegmentIndex - 1].columns();
            size_t segmentSampleCount = XSegments[XSegmentIndex].columns();
            casadi::MX dt = segmentDts(XSegmentIndex - 1);
            opti.subject_to(XSegments[XSegmentIndex - 1](ALL, previousSegmentSampleCount - 1)
                    + VSegments[XSegmentIndex](ALL, 0) * dt == XSegments[XSegmentIndex](ALL, 0));
            opti.subject_to(VSegments[XSegmentIndex - 1](ALL, previousSegmentSampleCount - 1)
                    + USegments[XSegmentIndex - 1](ALL, 0) * dt == VSegments[XSegmentIndex](ALL, 0));
            for (size_t sampleIndex = 1; sampleIndex < segmentSampleCount; sampleIndex++) {
                opti.subject_to(XSegments[XSegmentIndex](ALL, sampleIndex - 1) + VSegments[XSegmentIndex](ALL, sampleIndex) * dt
                        == XSegments[XSegmentIndex](ALL, sampleIndex));
                opti.subject_to(VSegments[XSegmentIndex](ALL, sampleIndex - 1) + USegments[XSegmentIndex - 1](ALL, sampleIndex) * dt
                        == VSegments[XSegmentIndex](ALL, sampleIndex));
            }
        }
    }

    HolonomicTrajectory CasADiHolonomicTrajectoryOptimizationProblem::ConstructTrajectory(const casadi::OptiSol& solution,
                const std::vector<casadi::MX>& xSegments, const std::vector<casadi::MX>& ySegments,
                const std::vector<casadi::MX>& thetaSegments, const std::vector<casadi::MX>& vxSegments,
                const std::vector<casadi::MX>& vySegments, const std::vector<casadi::MX>& omegaSegments,
                const casadi::MX& segmentDts) {

        std::vector<HolonomicTrajectorySegment> segments;
        segments.reserve(xSegments.size());

        for (size_t xSegmentIndex = 0; xSegmentIndex < xSegments.size(); xSegmentIndex++) {
            size_t segmentSampleCount = xSegments[xSegmentIndex].columns();
            double dt = xSegmentIndex == 0 ? 0.0 : static_cast<double>(solution.value(segmentDts(xSegmentIndex - 1)));
            std::vector<HolonomicTrajectorySample> samples;
            samples.reserve(segmentSampleCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < segmentSampleCount; segmentSampleIndex++) {
                samples.push_back(HolonomicTrajectorySample(
                        static_cast<double>(solution.value(xSegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(ySegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(thetaSegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(vxSegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(vySegments[xSegmentIndex](segmentSampleIndex))),
                        static_cast<double>(solution.value(omegaSegments[xSegmentIndex](segmentSampleIndex)))));
            }
            segments.push_back(HolonomicTrajectorySegment(dt, samples));
        }

        return HolonomicTrajectory(segments);
    }

    void CasADiHolonomicTrajectoryOptimizationProblem::ApplyHolonomicPathConstraints() {
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