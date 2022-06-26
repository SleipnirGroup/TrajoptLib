#pragma once

#include <vector>

namespace helixtrajectory {
	struct Waypoint {
		const double x, y, heading;
	};
	struct Path {
		const std::vector<Waypoint> waypoints;
		Path(const std::vector<Waypoint>& waypoints);
	};
}