#include "OptimalTrajectoryGenerator.h"

#include "optimization/CasADiOpti.h"
#include "DebugOptions.h"
#include "InvalidPathException.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "optimization/SwerveTrajectoryOptimizationProblem.h"
#include "trajectory/HolonomicTrajectory.h"

namespace helixtrajectory {

    HolonomicTrajectory OptimalTrajectoryGenerator::Generate(const SwerveDrivetrain& swerveDrivetrain,
            const HolonomicPath& holonomicPath, const HolonomicTrajectory* previousSolution) {
        if (!holonomicPath.IsValid()) {
            throw InvalidPathException("Cannot optimize an invalid path.");
        }
//         std::vector<HolonomicPath> splitPaths;
//         size_t splitIndex = 0;
//         for (size_t waypointIndex = 1; waypointIndex < holonomicPath.holonomicWaypoints.size(); waypointIndex++) {
//             if (holonomicPath.holonomicWaypoints[waypointIndex].IsSplitWaypoint()) {
//                 std::vector<HolonomicWaypoint> splitPathWaypoints;
//                 splitPathWaypoints.reserve(waypointIndex - splitIndex + 1);
//                 for (size_t splitPathIndex = splitIndex; splitPathIndex <= waypointIndex; splitPathIndex++) {
//                     splitPathWaypoints.push_back(holonomicPath.holonomicWaypoints[splitPathIndex]);
//                 }
//                 paths.push_back(HolonomicPath(splitPathWaypoints, holonomicPath.bumpers));
//                 splitIndex = waypointIndex;
//             }
//         }
//         std::vector<TrajectorySample> samples;
//         samples.reserve(holonomicPath.ControlIntervalTotal() + 1);
//         for (const HolonomicPath& path : paths) {
//             SwerveTrajectoryOptimizationProblem<CasADiOpti> problem(swerveDrivetrain, path);
//             std::vector<TrajectorySample> solvedSamples = problem.Generate().samples;
// #ifdef DEBUG_OUTPUT
//             std::cout << "Solution Found:" << std::endl;
//             problem.PrintSolution();
// #endif
//             for (TrajectorySample solvedSample : solvedSamples) {
//                 samples.push_back(solvedSample);
//             }
//         }
//         return Trajectory(samples);
        SwerveTrajectoryOptimizationProblem<CasADiOpti> problem(swerveDrivetrain, holonomicPath);
        return problem.Generate();
    }
}