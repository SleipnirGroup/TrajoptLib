#include "OptimalTrajectoryGenerator.h"

#include "CasADiOpti.h"
#include "DebugOptions.h"
#include "InvalidPathException.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "SwerveTrajectoryOptimizationProblem.h"

namespace helixtrajectory {

    Trajectory OptimalTrajectoryGenerator::Generate(const SwerveDrivetrain& swerveDrivetrain,
            const HolonomicPath& holonomicPath, const Trajectory* previousSolution) {
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
            SwerveTrajectoryOptimizationProblem<CasADiOpti> problem(swerveDrivetrain, holonomicPath);
            std::vector<HolonomicTrajectorySegment> solvedSegments = problem.Generate().holonomicSegments;
#ifdef DEBUG_OUTPUT
            std::cout << "Solution Found:" << std::endl;
            problem.PrintSolution();
#endif
            for (HolonomicTrajectorySegment solvedSegment : solvedSegments) {
                segments.push_back(solvedSegment);
            }
        }
        return HolonomicTrajectory(segments);
    }
}