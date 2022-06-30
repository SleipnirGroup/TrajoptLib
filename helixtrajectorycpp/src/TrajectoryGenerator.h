#pragma once

#include <memory>

#include "Path.h"
#include "SwerveDrive.h"
#include "Trajectory.h"

namespace helixtrajectory {
	class TrajectoryGenerator {
	private:
		const SwerveDrive* drive;
		void ApplyBoundryConstraints(casadi::Opti& solver, const casadi::MX& vx, const casadi::MX& vy, const casadi::MX& omega, const size_t nTotal);
		void ApplyWaypointConstraints(casadi::Opti& solver, const casadi::MX& x, const casadi::MX& y, const casadi::MX& theta,  const casadi::MX& vx, const casadi::MX& vy, const casadi::MX& omega, Path& path);
	public:
		TrajectoryGenerator(const SwerveDrive& drive);
		std::unique_ptr<Trajectory> Generate(Path& path);
	};
}