#include "HolonomicTrajectoryGenerator.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicDrive.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "Obstacle.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {

    HolonomicTrajectoryGenerator::HolonomicTrajectoryGenerator(const HolonomicDrive& drive, const HolonomicPath& path, const std::vector<Obstacle>& obstacles)
            : TrajectoryGenerator(drive, path, obstacles),
            holonomicDrive(drive), holonomicPath(path),
            V(opti.variable(3, nTotal + 1)), vx(V(0, ALL)), vy(V(1, ALL)), omega(V(2, ALL)),
            U(opti.variable(3, nTotal)), ax(U(0, ALL)), ay(U(1, ALL)), alpha(U(2, ALL)) {

        for (size_t i = 0; i < nTotal; i++) {
            casadi::MX xNext = X(ALL, i) + V(ALL, i) * trajectorySegmentDts(i / nPerTrajectorySegment);
            casadi::MX vNext = V(ALL, i) + U(ALL, i) * trajectorySegmentDts(i / nPerTrajectorySegment);
            opti.subject_to(X(ALL, i + 1) == xNext);
            opti.subject_to(V(ALL, i + 1) == vNext);
        }
        std::cout << "Applied Holonomic Kinematics Constraints" << std::endl;

        drive.ApplyKinematicsConstraints(opti, theta, vx, vy, omega, ax, ay, alpha, nTotal);
        std::cout << "Applied Swerve Dynamics Constraints" << std::endl;
        
        ApplyHolonomicPathConstraints();
        std::cout << "Applied Holonomic Path Constraints" << std::endl;
    }

    HolonomicTrajectoryGenerator::~HolonomicTrajectoryGenerator() {
    }

    std::unique_ptr<HolonomicTrajectory> HolonomicTrajectoryGenerator::Generate() {
        opti.solver("ipopt");
        std::cout << "Located IPOPT Plugin" << std::endl;
        auto solution = opti.solve();
        std::cout << "Solution Found" << std::endl;

        std::vector<double> ts;
        ts.reserve(nTotal + 1);
        double cumT = 0.0;
        for (int i = 0; i < trajectorySegmentCount; i++) {
            double t = static_cast<double>(solution.value(trajectorySegmentTs(i)));
            double dt = t / nPerTrajectorySegment;
            for (int j = 0; j < nPerTrajectorySegment; j++) {
                ts.push_back(cumT + j * dt);
            }
            cumT += t;
        }
        ts.push_back(cumT);

        auto solutionX = solution.value(x);
        auto solutionY = solution.value(y);
        auto solutionTheta = solution.value(theta);
        auto solutionVX = solution.value(vx);
        auto solutionVY = solution.value(vy);
        auto solutionOmega = solution.value(omega);
        std::vector<HolonomicTrajectorySample> samples;
        samples.reserve(nTotal + 1);
        for (int i = 0; i < nTotal + 1; i++) {
            samples.push_back(HolonomicTrajectorySample{
                ts[i], static_cast<double>(solutionX(i)), static_cast<double>(solutionY(i)),
                static_cast<double>(solutionTheta(i)), static_cast<double>(solutionVX(i)),
                static_cast<double>(solutionVY(i)), static_cast<double>(solutionOmega(i)) });
        }
        HolonomicTrajectory* traj = new HolonomicTrajectory(samples);
        return std::unique_ptr<HolonomicTrajectory>(traj);
    }

    void HolonomicTrajectoryGenerator::ApplyHolonomicPathConstraints() {
        for (int i = 0; i < waypointCount; i++) {
            int index = i * nPerTrajectorySegment;
            const HolonomicWaypoint& waypoint = holonomicPath.waypoints[i];
            if (waypoint.vMagnitudeConstrained) {
                if (waypoint.vxConstrained) {
                    opti.subject_to(vx(index) == waypoint.vx);
                }
                if (waypoint.vyConstrained) {
                    opti.subject_to(vy(index) == waypoint.vy);
                }
                if (!waypoint.vxConstrained && !waypoint.vyConstrained) {
                    if (waypoint.vx == 0.0 && waypoint.vy == 0.0) {
                        opti.subject_to(vx(index) == 0.0);
                        opti.subject_to(vy(index) == 0.0);
                    } else {
                        double magnitudeSquared = waypoint.vx * waypoint.vx + waypoint.vy * waypoint.vy;
                        opti.subject_to(vx(index) * vx(index) + vy(index) * vy(index) == magnitudeSquared);
                    }
                }
            } else if (waypoint.vxConstrained && waypoint.vyConstrained) {
                if (waypoint.vx == 0.0 && waypoint.vy == 0.0) {
                        opti.subject_to(vx(index) == 0.0);
                        opti.subject_to(vy(index) == 0.0);
                } else {
                    casadi::MX scalarMultiplier = opti.variable();
                    opti.subject_to(vx(index) == scalarMultiplier * waypoint.vx);
                    opti.subject_to(vy(index) == scalarMultiplier * waypoint.vy);
                }
            }

            if (waypoint.omegaConstrained) {
                opti.subject_to(omega(index) == waypoint.omega);
            }
        }
    }
}