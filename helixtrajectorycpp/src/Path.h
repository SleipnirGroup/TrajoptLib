#pragma once

#include <vector>

namespace helixtrajectory {
	struct Waypoint {
		const double x, y, heading, vx, vy, omega;
		const bool xConstrained, yConstrained, headingConstrained,
				vxConstrained, vyConstrained, omegaConstrained;
	};
	struct Path {
		const std::vector<Waypoint> waypoints;
		Path(const std::vector<Waypoint>& waypoints);
	};
}