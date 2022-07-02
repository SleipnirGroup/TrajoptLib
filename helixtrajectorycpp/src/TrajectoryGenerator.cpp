#include <iostream>

#include <casadi/casadi.hpp>

#include "TrajectoryGenerator.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {

	TrajectoryGenerator::TrajectoryGenerator(const SwerveDrive& drive)
		: drive(&drive) {}

	std::unique_ptr<Trajectory> TrajectoryGenerator::Generate(Path& path) {
		const size_t waypointCount = path.waypoints.size();
		const size_t trajectorySegmentCount = waypointCount - 1;
		const size_t nTotal = N_PER_TRAJECTORY_SEGMENT * trajectorySegmentCount;

		const casadi::Slice all;
		const casadi::Slice zeroOneTwo(0, 3);
		const casadi::Slice threeFourFive(3, 6);

		std::cout << "Initializing Optimizer" << std::endl;
		auto opti = casadi::Opti();

		std::vector<casadi::MX> trajectorySegmentTs;
		std::vector<casadi::MX> trajectorySegmentDts;
		trajectorySegmentTs.reserve(trajectorySegmentCount);
		trajectorySegmentDts.reserve(trajectorySegmentCount);
		casadi::MX totalT = 0;
		for (int i = 0; i < trajectorySegmentCount; i++) {
			casadi::MX t = opti.variable();
			casadi::MX dt = t / N_PER_TRAJECTORY_SEGMENT;
			trajectorySegmentTs.push_back(t);
			trajectorySegmentDts.push_back(dt);
			totalT += t;
			opti.subject_to(t >= 0);
			opti.set_initial(t, 5);
		}
		std::cout << "Applied Time Constraints" << std::endl;
		opti.minimize(totalT);
		std::cout << "Set Optimization Objective" << std::endl;

		casadi::MX X = opti.variable(6, nTotal + 1);

		casadi::MX x = X(0, all);
		casadi::MX y = X(1, all);
		casadi::MX theta = X(2, all);
		casadi::MX vx = X(3, all);
		casadi::MX vy = X(4, all);
		casadi::MX omega = X(5, all);

		casadi::MX U = opti.variable(3, nTotal);

		casadi::MX ax = U(0, all);
		casadi::MX ay = U(1, all);
		casadi::MX alpha = U(2, all);

		std::cout << "Set up kinematic variables" << std::endl;

		// TODO: maybe get rid of lambda and just use the slices below
		for (int i = 0; i < nTotal; i++) {
			casadi::MX xNext = X(all, i) + vertcat(X(threeFourFive, i), U(zeroOneTwo, i)) * trajectorySegmentDts[i / N_PER_TRAJECTORY_SEGMENT];
			opti.subject_to(X(all, i + 1) == xNext);
		}
		std::cout << "Applied kinematic constraints" << std::endl;

		opti.set_initial(X(zeroOneTwo, all), generateInitialTrajectory(path));
		std::cout << "Set initial trajectory" << std::endl;

		drive->ApplyKinematicsConstraints(opti, theta, vx, vy, omega, ax, ay, alpha, nTotal);
		std::cout << "Applied swerve constraints" << std::endl;
		ApplyBoundryConstraints(opti, vx, vy, omega, nTotal);
		std::cout << "Applied boundry constraints" << std::endl;
		ApplyWaypointConstraints(opti, x, y, theta, vx, vy, omega, path);
		std::cout << "Applied Waypoint constraints" << std::endl;

		opti.solver("ipopt");
		std::cout << "Set solver to ipopt" << std::endl;
		auto solution = opti.solve();
		std::cout << "Solution Found" << std::endl;

		std::vector<double> ts;
		ts.reserve(nTotal + 1);
		double cumT = 0.0;
		for (int i = 0; i < trajectorySegmentCount; i++) {
			double t = static_cast<double>(solution.value(trajectorySegmentTs[i]));
			double dt = t / N_PER_TRAJECTORY_SEGMENT;
			for (int j = 0; j < N_PER_TRAJECTORY_SEGMENT; j++) {
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
				ts[i], static_cast<double>(solutionX(i)), static_cast<double>(solutionY(i)), static_cast<double>(solutionTheta(i)),
			    static_cast<double>(solutionVX(i)), static_cast<double>(solutionVY(i)), static_cast<double>(solutionOmega(i)) });
		}
		Trajectory* traj = new Trajectory(samples);
		return std::unique_ptr<Trajectory>(traj);
	}

	void TrajectoryGenerator::ApplyBoundryConstraints(casadi::Opti& solver, const casadi::MX& vx, const casadi::MX& vy, const casadi::MX& omega, const size_t nTotal) {
		solver.subject_to(vx(0) == 0);
		solver.subject_to(vy(0) == 0);
		solver.subject_to(omega(0) == 0);
		solver.subject_to(vx(nTotal) == 0);
		solver.subject_to(vy(nTotal) == 0);
		solver.subject_to(omega(nTotal) == 0);
	}

	void TrajectoryGenerator::ApplyWaypointConstraints(casadi::Opti& solver, const casadi::MX& x, const casadi::MX& y, const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy, const casadi::MX& omega, Path& path) {
		const size_t waypointCount = path.waypoints.size();
		for (int i = 0; i < waypointCount; i++) {
			int index = i * N_PER_TRAJECTORY_SEGMENT;
			const Waypoint& waypoint = path.waypoints[i];
			if (waypoint.xConstrained) {
				solver.subject_to(x(index) == waypoint.x);
			}
			if (waypoint.yConstrained) {
				solver.subject_to(y(index) == waypoint.y);
			}
			if (waypoint.headingConstrained) {
				solver.subject_to(theta(index) == waypoint.heading);
			}
			if (waypoint.vMagnitudeConstrained) {
				if (waypoint.vxConstrained) {
					solver.subject_to(vx(index) == waypoint.vx);
				}
				if (waypoint.vyConstrained) {
					solver.subject_to(vy(index) == waypoint.vy);
				}
				if (!waypoint.vxConstrained && !waypoint.vyConstrained) {
					double magnitudeSquared = waypoint.vx * waypoint.vx + waypoint.vy * waypoint.vy;
					solver.subject_to(vx(index) * vx(index) + vy(index) * vy(index) == magnitudeSquared);
				}
			} else {
				casadi::MX scalarMultiplier = solver.variable();
				if (waypoint.vxConstrained) {
					solver.subject_to(vx(index) == scalarMultiplier * waypoint.vx);
				}
				if (waypoint.vyConstrained) {
					solver.subject_to(vy(index) == scalarMultiplier * waypoint.vy);
				}
			}
			
			if (waypoint.omegaConstrained) {
				solver.subject_to(omega(index) == waypoint.omega);
			}
		}
	}
}