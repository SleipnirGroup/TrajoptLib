#pragma once

#include <vector>

namespace helixtrajectory {
	struct TrajectorySample {
		const double ts, x, y, heading, vx, vy, omega;
	};

	struct Trajectory {
		const std::vector<TrajectorySample> samples;

		Trajectory(const std::vector<TrajectorySample>& samples);
	};
}