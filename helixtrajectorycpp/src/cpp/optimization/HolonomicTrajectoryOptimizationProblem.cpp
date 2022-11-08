#include "DebugOptions.h"

#include "HolonomicTrajectoryOptimizationProblem.h"

#include <iostream>
#include <vector>

#include "CasADiOpti.h"
#include "HolonomicDrivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "HolonomicTrajectorySample.h"
#include "HolonomicTrajectorySegment.h"
#include "HolonomicWaypoint.h"
#include "Obstacle.h"
#include "TrajectoryGenerationException.h"

namespace helixtrajectory {

    template<typename Opti>
    HolonomicTrajectoryOptimizationProblem<Opti>::HolonomicTrajectoryOptimizationProblem(
            const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath)
            : TrajectoryOptimizationProblem<Opti>(holonomicDrivetrain, holonomicPath),
            holonomicDrivetrain(holonomicDrivetrain), holonomicPath(holonomicPath),
            vx(), vy(), omega(),
            ax(), ay(), alpha(),
            vxSegments(), vySegments(), omegaSegments() {

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

        vxSegments.push_back({vx[0]});
        vySegments.push_back({vy[0]});
        omegaSegments.push_back({omega[0]});

        size_t sampleIndex = 1;
        for (size_t waypointIndex = 1; waypointIndex < TrajectoryOptimizationProblem<Opti>::waypointCount; waypointIndex++) {
            size_t controlIntervalCount = HolonomicTrajectoryOptimizationProblem::holonomicPath
                    .holonomicWaypoints[waypointIndex].controlIntervalCount;
            std::vector<Expression> vxSegment;
            std::vector<Expression> vySegment;
            std::vector<Expression> omegaSegment;
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

        ApplyHolonomicWaypointConstraints(
                TrajectoryOptimizationProblem<Opti>::opti,
                vxSegments,
                vySegments,
                omegaSegments,
                HolonomicTrajectoryOptimizationProblem::holonomicPath);
#ifdef DEBUG_OUTPUT
        std::cout << "Applied Holonomic Path Constraints" << std::endl;
#endif
    }

    template<typename Opti>
    HolonomicTrajectory HolonomicTrajectoryOptimizationProblem<Opti>::Generate() {
        try {
            TrajectoryOptimizationProblem<Opti>::opti.Solve();
#ifdef DEBUG_OUTPUT
            std::cout << "Solution stored" << std::endl;
#endif
            return ConstructTrajectory(TrajectoryOptimizationProblem<Opti>::opti,
                    TrajectoryOptimizationProblem<Opti>::dtSegments,
                    TrajectoryOptimizationProblem<Opti>::xSegments,
                    TrajectoryOptimizationProblem<Opti>::ySegments,
                    TrajectoryOptimizationProblem<Opti>::thetaSegments,
                    vxSegments, vySegments, omegaSegments);
        } catch (...) {
            throw TrajectoryGenerationException("Error optimizing trajectory!");
        }
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
    void HolonomicTrajectoryOptimizationProblem<Opti>::ApplyHolonomicWaypointConstraints(Opti& opti,
            const std::vector<std::vector<Expression>>& vxSegments, const std::vector<std::vector<Expression>>& vySegments,
            const std::vector<std::vector<Expression>>& omegaSegments, const HolonomicPath& holonomicPath) {
        size_t waypointCount = vxSegments.size();
        for (size_t waypointIndex = 0; waypointIndex < waypointCount; waypointIndex++) {
            const HolonomicWaypoint& waypoint = holonomicPath.holonomicWaypoints[waypointIndex];
            size_t sampleIndex = vxSegments[waypointIndex].size() - 1;
            // std::cout << "sample index for waypoint constraints: " << sampleIndex << std::endl;
            if (waypoint.velocityMagnitudeConstrained) {
                if (waypoint.velocityXConstrained) {
                    opti.SubjectTo(vxSegments[waypointIndex][sampleIndex] == waypoint.velocityX);
                }
                if (waypoint.velocityYConstrained) {
                    opti.SubjectTo(vySegments[waypointIndex][sampleIndex] == waypoint.velocityY);
                }
                if (!waypoint.velocityXConstrained && !waypoint.velocityYConstrained) {
                    if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.SubjectTo(vxSegments[waypointIndex][sampleIndex] == 0.0);
                        opti.SubjectTo(vySegments[waypointIndex][sampleIndex] == 0.0);
                    } else {
                        double magnitudeSquared = waypoint.velocityX * waypoint.velocityX + waypoint.velocityY * waypoint.velocityY;
                        opti.SubjectTo(vxSegments[waypointIndex][sampleIndex] * vxSegments[waypointIndex][sampleIndex]
                                      + vySegments[waypointIndex][sampleIndex] * vySegments[waypointIndex][sampleIndex] == magnitudeSquared);
                    }
                }
            } else if (waypoint.velocityXConstrained && waypoint.velocityYConstrained) {
                if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.SubjectTo(vxSegments[waypointIndex][sampleIndex] == 0.0);
                        opti.SubjectTo(vySegments[waypointIndex][sampleIndex] == 0.0);
                } else {
                    Expression scalarMultiplier = opti.Variable();
                    opti.SubjectTo(vxSegments[waypointIndex][sampleIndex] == scalarMultiplier * waypoint.velocityX);
                    opti.SubjectTo(vySegments[waypointIndex][sampleIndex] == scalarMultiplier * waypoint.velocityY);
                }
            }
            if (waypoint.angularVelocityConstrained) {
                opti.SubjectTo(omegaSegments[waypointIndex][sampleIndex] == waypoint.angularVelocity);
            }
        }
    }

    template<typename Opti>
    HolonomicTrajectory HolonomicTrajectoryOptimizationProblem<Opti>::ConstructTrajectory(const Opti& opti,
            const std::vector<std::vector<Expression>>& dtSegments,
            const std::vector<std::vector<Expression>>& xSegments, const std::vector<std::vector<Expression>>& ySegments,
            const std::vector<std::vector<Expression>>& thetaSegments, const std::vector<std::vector<Expression>>& vxSegments,
            const std::vector<std::vector<Expression>>& vySegments, const std::vector<std::vector<Expression>>& omegaSegments) {

        std::vector<HolonomicTrajectorySegment> segments;
        segments.reserve(xSegments.size());

        segments.push_back(HolonomicTrajectorySegment({HolonomicTrajectorySample(
                0.0,
                static_cast<double>(opti.SolutionValue(xSegments[0][0])),
                static_cast<double>(opti.SolutionValue(ySegments[0][0])),
                static_cast<double>(opti.SolutionValue(thetaSegments[0][0])),
                static_cast<double>(opti.SolutionValue(vxSegments[0][0])),
                static_cast<double>(opti.SolutionValue(vySegments[0][0])),
                static_cast<double>(opti.SolutionValue(omegaSegments[0][0])))}));

        for (size_t xSegmentIndex = 1; xSegmentIndex < xSegments.size(); xSegmentIndex++) {
            size_t segmentSampleCount = xSegments[xSegmentIndex].size();
            std::vector<HolonomicTrajectorySample> samples;
            samples.reserve(segmentSampleCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < segmentSampleCount; segmentSampleIndex++) {
                samples.push_back(HolonomicTrajectorySample(
                        static_cast<double>(opti.SolutionValue(dtSegments[xSegmentIndex - 1][segmentSampleIndex])),
                        static_cast<double>(opti.SolutionValue(xSegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(opti.SolutionValue(ySegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(opti.SolutionValue(thetaSegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(opti.SolutionValue(vxSegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(opti.SolutionValue(vySegments[xSegmentIndex][segmentSampleIndex])),
                        static_cast<double>(opti.SolutionValue(omegaSegments[xSegmentIndex][segmentSampleIndex]))));
            }
            segments.push_back(HolonomicTrajectorySegment(samples));
        }

        return HolonomicTrajectory(segments);
    }

    template class HolonomicTrajectoryOptimizationProblem<CasADiOpti>;
}