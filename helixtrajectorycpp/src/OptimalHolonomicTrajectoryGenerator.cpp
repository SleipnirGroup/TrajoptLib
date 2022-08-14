#include "OptimalHolonomicTrajectoryGenerator.h"

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
#include "TrajectoryUtil.h"

namespace helixtrajectory {

    OptimalHolonomicTrajectoryGenerator::OptimalHolonomicTrajectoryGenerator(const HolonomicDrivetrain& holonomicDrivetrain, const HolonomicPath& holonomicPath, const std::vector<Obstacle>& obstacles)
            : OptimalTrajectoryGenerator(holonomicDrivetrain, holonomicPath, obstacles),
            holonomicDrivetrain(holonomicDrivetrain), holonomicPath(holonomicPath),
            V(opti.variable(3, controlIntervalTotal + 1)), vx(V(0, ALL)), vy(V(1, ALL)), omega(V(2, ALL)),
            U(opti.variable(3, controlIntervalTotal)), ax(U(0, ALL)), ay(U(1, ALL)), alpha(U(2, ALL)) {

        size_t sampleIndex = OptimalHolonomicTrajectoryGenerator::holonomicPath
                .holonomicWaypoints[0].controlIntervalCount;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t segmentIntervalCount = OptimalHolonomicTrajectoryGenerator::holonomicPath
                    .holonomicWaypoints[waypointIndex].controlIntervalCount;
            casadi::MX dt = trajectorySegmentDts(waypointIndex - 1);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < segmentIntervalCount; segmentSampleIndex++) {
                casadi::MX XNext = X(ALL, sampleIndex) + V(ALL, sampleIndex) * dt;
                casadi::MX VNext = V(ALL, sampleIndex) + U(ALL, sampleIndex) * dt;
                opti.subject_to(X(ALL, sampleIndex + 1) == XNext);
                opti.subject_to(V(ALL, sampleIndex + 1) == VNext);
                sampleIndex++;
            }
        }
        std::cout << "Applied Holonomic Kinematics Constraints" << std::endl;

        OptimalHolonomicTrajectoryGenerator::holonomicDrivetrain.ApplyKinematicsConstraints(
                opti, theta, vx, vy, omega, ax, ay, alpha, controlIntervalTotal);
        std::cout << "Applied Swerve Dynamics Constraints" << std::endl;
        
        ApplyHolonomicPathConstraints();
        std::cout << "Applied Holonomic Path Constraints" << std::endl;
    }

    OptimalHolonomicTrajectoryGenerator::~OptimalHolonomicTrajectoryGenerator() {
    }

    HolonomicTrajectory OptimalHolonomicTrajectoryGenerator::Generate() {
        opti.solver("ipopt");
        std::cout << "Located IPOPT Plugin" << std::endl;
        auto solution = opti.solve();
        std::cout << "Solution Found" << std::endl;

        std::vector<double> solutionTimestamp;
        solutionTimestamp.reserve(controlIntervalTotal + 1);
        double cumT = 0.0;
        for (size_t segmentIndex = 0; segmentIndex < trajectorySegmentCount; segmentIndex++) {
            double t = static_cast<double>(solution.value(trajectorySegmentTs(segmentIndex)));
            size_t controlIntervalCount = path.GetWaypoint(segmentIndex + 1).controlIntervalCount;
            double dt = t / controlIntervalCount;
            for (size_t intervalIndex = 0; intervalIndex < controlIntervalCount; intervalIndex++) {
                solutionTimestamp.push_back(cumT + segmentIndex * dt);
            }
            cumT += t;
        }
        solutionTimestamp.push_back(cumT);

        auto solutionDts = solution.value(trajectorySegmentDts);
        auto solutionX = solution.value(x);
        auto solutionY = solution.value(y);
        auto solutionTheta = solution.value(theta);
        auto solutionVX = solution.value(vx);
        auto solutionVY = solution.value(vy);
        auto solutionOmega = solution.value(omega);

        std::vector<HolonomicTrajectorySegment> segments;
        segments.reserve(waypointCount);
        const HolonomicWaypoint& firstWaypoint = holonomicPath.holonomicWaypoints[0];
        segments.push_back(HolonomicTrajectorySegment(0.0,
                {HolonomicTrajectorySample(static_cast<double>(solutionX(0)), static_cast<double>(solutionY(0)),
                static_cast<double>(solutionTheta(0)), static_cast<double>(solutionVX(0)),
                static_cast<double>(solutionVY(0)), static_cast<double>(solutionOmega(0)))}));

        size_t sampleIndex = 0;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t controlIntervalCount = holonomicPath.holonomicWaypoints[waypointIndex].controlIntervalCount;
            double dt = static_cast<double>(solutionDts(waypointIndex - 1));
            std::vector<HolonomicTrajectorySample> samples;
            samples.reserve(controlIntervalCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < controlIntervalCount; segmentSampleIndex++) {
                samples.push_back(HolonomicTrajectorySample(static_cast<double>(solutionX(sampleIndex)), static_cast<double>(solutionY(sampleIndex)),
                static_cast<double>(solutionTheta(sampleIndex)), static_cast<double>(solutionVX(sampleIndex)),
                static_cast<double>(solutionVY(sampleIndex)), static_cast<double>(solutionOmega(i))))
            }
            segments.push_back(HolonomicTrajectorySegment(dt, samples));
        }

        std::vector<HolonomicTrajectorySample> samples;
        samples.reserve(controlIntervalTotal + 1);
        for (int i = 0; i < controlIntervalTotal + 1; i++) {
            samples.push_back(HolonomicTrajectorySample{
                solutionTimestamp[i], static_cast<double>(solutionX(i)), static_cast<double>(solutionY(i)),
                static_cast<double>(solutionTheta(i)), static_cast<double>(solutionVX(i)),
                static_cast<double>(solutionVY(i)), static_cast<double>(solutionOmega(i)) });
        }
        return HolonomicTrajectory(samples);
    }

    void OptimalHolonomicTrajectoryGenerator::ApplyHolonomicPathConstraints() {
        size_t sampleIndex = 0;
        for (size_t waypointIndex = 0; waypointIndex < waypointCount; waypointIndex++) {
            const HolonomicWaypoint& waypoint = holonomicPath.waypoints[waypointIndex];
            sampleIndex += waypoint.controlIntervalCount;
            if (waypoint.velocityMagnitudeConstrained) {
                if (waypoint.velocityXConstrained) {
                    opti.subject_to(vx(sampleIndex) == waypoint.velocityX);
                }
                if (waypoint.velocityYConstrained) {
                    opti.subject_to(vy(sampleIndex) == waypoint.velocityY);
                }
                if (!waypoint.velocityXConstrained && !waypoint.velocityYConstrained) {
                    if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.subject_to(vx(sampleIndex) == 0.0);
                        opti.subject_to(vy(sampleIndex) == 0.0);
                    } else {
                        double magnitudeSquared = waypoint.velocityX * waypoint.velocityX + waypoint.velocityY * waypoint.velocityY;
                        opti.subject_to(vx(sampleIndex) * vx(sampleIndex) + vy(sampleIndex) * vy(sampleIndex) == magnitudeSquared);
                    }
                }
            } else if (waypoint.velocityXConstrained && waypoint.velocityYConstrained) {
                if (waypoint.velocityX == 0.0 && waypoint.velocityY == 0.0) {
                        opti.subject_to(vx(sampleIndex) == 0.0);
                        opti.subject_to(vy(sampleIndex) == 0.0);
                } else {
                    casadi::MX scalarMultiplier = opti.variable();
                    opti.subject_to(vx(sampleIndex) == scalarMultiplier * waypoint.velocityX);
                    opti.subject_to(vy(sampleIndex) == scalarMultiplier * waypoint.velocityY);
                }
            }
            if (waypoint.angularVelocityConstrained) {
                opti.subject_to(omega(sampleIndex) == waypoint.angularVelocityConstrained);
            }
        }
    }
}