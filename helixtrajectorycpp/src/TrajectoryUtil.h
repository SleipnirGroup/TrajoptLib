#pragma once

#include <vector>

#include "casadi/casadi.hpp"

#include "Path.h"
#include "Trajectory.h"

namespace helixtrajectory {

	const int N_PER_TRAJECTORY_SEGMENT = 100;

    template<typename T>
    std::vector<double> linspace(T start_in, T end_in, int num_in);

	casadi::DM generateInitialTrajectory(Path& path);

	void printPath(Path& path);
	void printTrajectory(Trajectory& trajectory);
}