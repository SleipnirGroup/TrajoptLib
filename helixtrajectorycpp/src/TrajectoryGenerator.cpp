#include <iostream>

#include <casadi/casadi.hpp>

#include "TrajectoryGenerator.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {

    TrajectoryGenerator::TrajectoryGenerator(const SwerveDrive& drive, const Path& path, const std::vector<Obstacle>& obstacles)
            : drive(drive), path(path), obstacles(obstacles),
            waypointCount(path.waypoints.size()), trajectorySegmentCount(waypointCount - 1), 
            nPerTrajectorySegment(100), nTotal(nPerTrajectorySegment * trajectorySegmentCount),
            opti(), X(opti.variable(6, nTotal + 1)), x(X(0, all)), y(X(1, all)), theta(X(2, all)),
            vx(X(3, all)), vy(X(4, all)), omega(X(5, all)), U(opti.variable(3, nTotal)),
            ax(U(0, all)), ay(U(1, all)), alpha(U(2, all)),
            trajectorySegmentTs(opti.variable(1, trajectorySegmentCount)),
            trajectorySegmentDts(trajectorySegmentTs / nPerTrajectorySegment) {
        casadi::MX totalT = 0;
        for (size_t i = 0; i < trajectorySegmentCount; i++) {
            totalT += trajectorySegmentTs(i);
            opti.subject_to(trajectorySegmentTs(i) >= 0);
            opti.set_initial(trajectorySegmentTs(i), 5);
        }
        std::cout << "Applied Time Constraints" << std::endl;
        opti.minimize(totalT);
        std::cout << "Set Optimization Objective" << std::endl;

        for (size_t i = 0; i < nTotal; i++) {
            casadi::MX xNext = X(all, i) + vertcat(X(threeFourFive, i), U(zeroOneTwo, i)) * trajectorySegmentDts(i / nPerTrajectorySegment);
            opti.subject_to(X(all, i + 1) == xNext);
        }
        std::cout << "Applied kinematic constraints" << std::endl;

        opti.set_initial(X(zeroOneTwo, all), generateInitialTrajectory(path, nPerTrajectorySegment));
        std::cout << "Set initial trajectory" << std::endl;

        drive.ApplyKinematicsConstraints(opti, theta, vx, vy, omega, ax, ay, alpha, nTotal);
        std::cout << "Applied Obstacle constraints" << std::endl;
        drive.ApplyObstacleConstraints(opti, x, y, theta, nTotal, obstacles);
        std::cout << "Applied swerve constraints" << std::endl;
        ApplyBoundryConstraints();
        std::cout << "Applied boundry constraints" << std::endl;
        ApplyWaypointConstraints();
        std::cout << "Applied Waypoint constraints" << std::endl;
    }

    std::unique_ptr<Trajectory> TrajectoryGenerator::Generate() {
        opti.solver("ipopt");
        std::cout << "Set solver to ipopt" << std::endl;
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
        std::vector<TrajectorySample> samples;
        samples.reserve(nTotal + 1);
        for (int i = 0; i < nTotal + 1; i++) {
            samples.push_back(TrajectorySample{
                ts[i], static_cast<double>(solutionX(i)), static_cast<double>(solutionY(i)),
                static_cast<double>(solutionTheta(i)), static_cast<double>(solutionVX(i)),
                static_cast<double>(solutionVY(i)), static_cast<double>(solutionOmega(i)) });
        }
        Trajectory* traj = new Trajectory(samples);
        return std::unique_ptr<Trajectory>(traj);
    }

    void TrajectoryGenerator::ApplyBoundryConstraints() {
        opti.subject_to(vx(0) == 0);
        opti.subject_to(vy(0) == 0);
        opti.subject_to(omega(0) == 0);
        opti.subject_to(vx(nTotal) == 0);
        opti.subject_to(vy(nTotal) == 0);
        opti.subject_to(omega(nTotal) == 0);
    }

    void TrajectoryGenerator::ApplyWaypointConstraints() {
        for (int i = 0; i < waypointCount; i++) {
            int index = i * nPerTrajectorySegment;
            const Waypoint& waypoint = path.waypoints[i];
            if (waypoint.xConstrained) {
                opti.subject_to(x(index) == waypoint.x);
            }
            if (waypoint.yConstrained) {
                opti.subject_to(y(index) == waypoint.y);
            }
            if (waypoint.headingConstrained) {
                opti.subject_to(theta(index) == waypoint.heading);
            }
            if (waypoint.vMagnitudeConstrained) {
                if (waypoint.vxConstrained) {
                    opti.subject_to(vx(index) == waypoint.vx);
                }
                if (waypoint.vyConstrained) {
                    opti.subject_to(vy(index) == waypoint.vy);
                }
                if (!waypoint.vxConstrained && !waypoint.vyConstrained) {
                    double magnitudeSquared = waypoint.vx * waypoint.vx + waypoint.vy * waypoint.vy;
                    opti.subject_to(vx(index) * vx(index) + vy(index) * vy(index) == magnitudeSquared);
                }
            } else {
                casadi::MX scalarMultiplier = opti.variable();
                if (waypoint.vxConstrained) {
                    opti.subject_to(vx(index) == scalarMultiplier * waypoint.vx);
                }
                if (waypoint.vyConstrained) {
                    opti.subject_to(vy(index) == scalarMultiplier * waypoint.vy);
                }
            }

            if (waypoint.omegaConstrained) {
                opti.subject_to(omega(index) == waypoint.omega);
            }
        }
    }
}