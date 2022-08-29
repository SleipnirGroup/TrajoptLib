#include "OptimalTrajectoryGenerator.h"

#include "CasADiSwerveTrajectoryOptimizationProblem.h"
#include "InvalidPathException.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    HolonomicTrajectory OptimalTrajectoryGenerator::Generate(const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath) {
        if (!holonomicPath.IsValid()) {
            throw InvalidPathException("Cannot optimize an invalid path.");
        }
        std::vector<HolonomicPath> paths;
        size_t splitIndex = 0;
        for (size_t waypointIndex = 1; waypointIndex < holonomicPath.holonomicWaypoints.size(); waypointIndex++) {
            if (holonomicPath.holonomicWaypoints[waypointIndex].IsSplitWaypoint()) {
                std::vector<HolonomicWaypoint> splitPathWaypoints;
                splitPathWaypoints.reserve(waypointIndex - splitIndex + 1);
                for (size_t splitPathIndex = splitIndex; splitPathIndex <= waypointIndex; splitPathIndex++) {
                    splitPathWaypoints.push_back(holonomicPath.holonomicWaypoints[splitPathIndex]);
                }
                paths.push_back(HolonomicPath(splitPathWaypoints));
                splitIndex = waypointIndex;
            }
        }
        std::vector<HolonomicTrajectorySegment> segments;
        segments.reserve(holonomicPath.holonomicWaypoints.size());
        for (const HolonomicPath& path : paths) {
            CasADiSwerveTrajectoryOptimizationProblem problem(swerveDrivetrain, holonomicPath);
            std::vector<HolonomicTrajectorySegment> solvedSegments = problem.Generate().holonomicSegments;
            for (HolonomicTrajectorySegment solvedSegment : solvedSegments) {
                segments.push_back(solvedSegment);
            }
        }
        return HolonomicTrajectory(segments);
    }
}