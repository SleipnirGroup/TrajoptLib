#include "DebugOptions.h"

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
            vx(), vy(), omega(),
            ax(), ay(), alpha(),
            vxSegments(), vySegments(), omegaSegments() {

        vx.reserve(sampleTotal);
        vy.reserve(sampleTotal);
        omega.reserve(sampleTotal);
        ax.reserve(controlIntervalTotal);
        ay.reserve(controlIntervalTotal);
        alpha.reserve(controlIntervalTotal);

        for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
            std::cout << "Setting up holonomic: " << std::endl;
            vx.push_back(opti.variable());
            vy.push_back(opti.variable());
            omega.push_back(opti.variable());
        }
        std::cout << "Created new velocity variables" << std::endl;
        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            ax.push_back(opti.variable());
            ay.push_back(opti.variable());
            alpha.push_back(opti.variable());
        }
        std::cout << "Created new acceleration variables" << std::endl;

        vxSegments.reserve(waypointCount);
        vySegments.reserve(waypointCount);
        omegaSegments.reserve(waypointCount);

        vxSegments.push_back({vx[0]});
        vySegments.push_back({vy[0]});
        omegaSegments.push_back({omega[0]});

        size_t sampleIndex = 1;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t controlIntervalCount = CasADiHolonomicTrajectoryOptimizationProblem::holonomicPath
                    .holonomicWaypoints[waypointIndex].controlIntervalCount;
            std::vector<casadi::MX> vxSegment;
            std::vector<casadi::MX> vySegment;
            std::vector<casadi::MX> omegaSegment;
            vxSegment.reserve(controlIntervalCount);
            vySegment.reserve(controlIntervalCount);
            omegaSegment.reserve(controlIntervalCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < controlIntervalCount; segmentSampleIndex++) {
                vxSegment.push_back(vx[sampleIndex + segmentSampleIndex]);
                vySegment.push_back(vy[sampleIndex + segmentSampleIndex]);
                omegaSegment.push_back(omega[sampleIndex + segmentSampleIndex]);
            }
            vxSegments.push_back(vxSegment);
            vySegments.push_back(vySegment);
            omegaSegments.push_back(omegaSegment);
            sampleIndex += controlIntervalCount;
        }

        ApplyKinematicsConstraints(opti, dt, x, y, theta, vx, vy, omega, ax, ay, alpha);
#ifdef DEBUG_OUTPUT
        std::cout << "Applied Holonomic Kinematics Constraints" << std::endl;
#endif

        ApplyHolonomicWaypointConstraints(opti, vxSegments, vySegments, omegaSegments,
                CasADiHolonomicTrajectoryOptimizationProblem::holonomicPath);
#ifdef DEBUG_OUTPUT
        std::cout << "Applied Holonomic Path Constraints" << std::endl;
#endif
    }

    HolonomicTrajectory CasADiHolonomicTrajectoryOptimizationProblem::Generate() {
        // I don't try-catch this next line since it should always work.
        // I'm assuming the dynamic lib is on the path and casadi can find it.
#ifdef DEBUG_OUTPUT
        opti.solver("ipopt");
        std::cout << "Located IPOPT Plugin" << std::endl;
#else
        auto pluginOptions = casadi::Dict();
        pluginOptions["ipopt.print_level"] = 0;
        pluginOptions["print_time"] = 0;
        pluginOptions["ipopt.sb"] = "yes";
        opti.solver("ipopt", pluginOptions);
#endif
        try {
            auto solution = opti.solve();
#ifdef DEBUG_OUTPUT
            std::cout << "Solution Found:" << std::endl;
            PrintSolution(solution);
#endif
            return ConstructTrajectory(solution, dtSegments, xSegments, ySegments, thetaSegments,
                vxSegments, vySegments, omegaSegments);
        } catch (...) {
            throw TrajectoryGenerationException("Error optimizing trajectory!");
        }
    }

    void CasADiHolonomicTrajectoryOptimizationProblem::ApplyKinematicsConstraints(casadi::Opti& opti,
            const std::vector<casadi::MX>& dt,
            const std::vector<casadi::MX>& x, const std::vector<casadi::MX>& y, const std::vector<casadi::MX>& theta,
            const std::vector<casadi::MX>& vx, const std::vector<casadi::MX>& vy, const std::vector<casadi::MX>& omega,
            const std::vector<casadi::MX>& ax, const std::vector<casadi::MX>& ay, const std::vector<casadi::MX>& alpha) {
        size_t sampleTotal = x.size();
        for (size_t sampleIndex = 1; sampleIndex < sampleTotal; sampleIndex++) {
            casadi::MX sampleDT = dt[sampleIndex - 1];
            opti.subject_to(x[sampleIndex - 1] + vx[sampleIndex] * sampleDT == x[sampleIndex]);
            opti.subject_to(y[sampleIndex - 1] + vy[sampleIndex] * sampleDT == y[sampleIndex]);
            opti.subject_to(theta[sampleIndex - 1] + omega[sampleIndex] * sampleDT == theta[sampleIndex]);
            opti.subject_to(vx[sampleIndex - 1] + ax[sampleIndex - 1] * sampleDT == vx[sampleIndex]);
            opti.subject_to(vy[sampleIndex - 1] + ay[sampleIndex - 1] * sampleDT == vy[sampleIndex]);
            opti.subject_to(omega[sampleIndex - 1] + alpha[sampleIndex - 1] * sampleDT == omega[sampleIndex]);
        }
    }

    void CasADiHolonomicTrajectoryOptimizationProblem::ApplyHolonomicWaypointConstraints(casadi::Opti& opti,
            const std::vector<std::vector<casadi::MX>>& vxSegments, const std::vector<std::vector<casadi::MX>>& vySegments,
            const std::vector<std::vector<casadi::MX>>& omegaSegments, const HolonomicPath& holonomicPath) {
        size_t waypointCount = vxSegments.size();
        for (size_t waypointIndex = 0; waypointIndex < waypointCount; waypointIndex++) {
            const HolonomicWaypoint& waypoint = holonomicPath.holonomicWaypoints[waypointIndex];
            size_t sampleIndex = vxSegments[waypointIndex].size() - 1;
            std::cout << "sample index for waypoint constraints: " << sampleIndex << std::endl;
            if (waypoint.velocityMagnitudeConstrained) {
                if (waypoint.velocityXConstrained) {
                    opti.subject_to(vxSegments[waypointIndex][sampleIndex] == waypoint.velocityX);
                }
                if (waypoint.velocityYConstrained) {
                    opti.subject_to(vySegments[waypointIndex][sampleIndex] == waypoint.velocityY);
                }
                if (!waypoint.velocityXConstrained && !waypoint.velocityYConstrained) {
                    if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.subject_to(vxSegments[waypointIndex][sampleIndex] == 0.0);
                        opti.subject_to(vySegments[waypointIndex][sampleIndex] == 0.0);
                    } else {
                        double magnitudeSquared = waypoint.velocityX * waypoint.velocityX + waypoint.velocityY * waypoint.velocityY;
                        opti.subject_to(vxSegments[waypointIndex][sampleIndex] * vxSegments[waypointIndex][sampleIndex]
                                      + vySegments[waypointIndex][sampleIndex] * vySegments[waypointIndex][sampleIndex] == magnitudeSquared);
                    }
                }
            } else if (waypoint.velocityXConstrained && waypoint.velocityYConstrained) {
                if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.subject_to(vxSegments[waypointIndex][sampleIndex] == 0.0);
                        opti.subject_to(vySegments[waypointIndex][sampleIndex] == 0.0);
                } else {
                    casadi::MX scalarMultiplier = opti.variable();
                    opti.subject_to(vxSegments[waypointIndex][sampleIndex] == scalarMultiplier * waypoint.velocityX);
                    opti.subject_to(vySegments[waypointIndex][sampleIndex] == scalarMultiplier * waypoint.velocityY);
                }
            }
            if (waypoint.angularVelocityConstrained) {
                opti.subject_to(omegaSegments[waypointIndex][sampleIndex] == waypoint.angularVelocity);
            }
        }
    }

    HolonomicTrajectory CasADiHolonomicTrajectoryOptimizationProblem::ConstructTrajectory(const casadi::OptiSol& solution,
            const std::vector<std::vector<casadi::MX>>& dtSegments,
            const std::vector<std::vector<casadi::MX>>& xSegments, const std::vector<std::vector<casadi::MX>>& ySegments,
            const std::vector<std::vector<casadi::MX>>& thetaSegments, const std::vector<std::vector<casadi::MX>>& vxSegments,
            const std::vector<std::vector<casadi::MX>>& vySegments, const std::vector<std::vector<casadi::MX>>& omegaSegments) {

        std::vector<HolonomicTrajectorySegment> segments;
        segments.reserve(xSegments.size());

        segments.push_back(HolonomicTrajectorySegment({HolonomicTrajectorySample(
                0.0,
                static_cast<double>(solution.value(xSegments[0][0])),
                static_cast<double>(solution.value(ySegments[0][0])),
                static_cast<double>(solution.value(thetaSegments[0][0])),
                static_cast<double>(solution.value(vxSegments[0][0])),
                static_cast<double>(solution.value(vySegments[0][0])),
                static_cast<double>(solution.value(omegaSegments[0][0])))}));

        for (size_t xSegmentIndex = 1; xSegmentIndex < xSegments.size(); xSegmentIndex++) {
            size_t segmentSampleCount = xSegments[xSegmentIndex].size();
            std::vector<HolonomicTrajectorySample> samples;
            samples.reserve(segmentSampleCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < segmentSampleCount; segmentSampleIndex++) {
                samples.push_back(HolonomicTrajectorySample(
                        static_cast<double>(solution.value(dtSegments[xSegmentIndex - 1][segmentSampleIndex])),
                        static_cast<double>(solution.value(xSegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(solution.value(ySegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(solution.value(thetaSegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(solution.value(vxSegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(solution.value(vySegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(solution.value(omegaSegments[xSegmentIndex][segmentSampleIndex]))));
            }
            segments.push_back(HolonomicTrajectorySegment(samples));
        }

        return HolonomicTrajectory(segments);
    }
}