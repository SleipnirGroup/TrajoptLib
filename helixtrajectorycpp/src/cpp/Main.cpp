#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "path/HolonomicPath.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"
#include "OptimalTrajectoryGenerator.h"

int main() {

    // auto opti = casadi::Opti();

    // auto variable1 = opti.variable(4, 1);
    // auto variable2 = opti.variable(4, 1);

    // std::cout << "pose    : " << variable1 << "\n";
    // std::cout << "variable: " << variable2 << std::endl;
    // std::cout << "pose(0): " << pose(slice0) << "\n";
    // std::cout << "pose(1): " << pose(slice1) << std::endl;

    using namespace helixtrajectory;
    SwerveDrivetrain swerveDrivetrain(45, 6,
            {SwerveModule(+0.6, +0.6, 0.04, 70, 2),
             SwerveModule(+0.6, -0.6, 0.04, 70, 2),
             SwerveModule(-0.6, +0.6, 0.04, 70, 2),
             SwerveModule(-0.6, -0.6, 0.04, 70, 2)});

    HolonomicPath holonomicPath(HolonomicPath({
        HolonomicWaypoint(PositionConstraint(0,    { 4,  0}), PositionConstraint(), HolonomicVelocityConstraint(0.0, {0.0, 0.0})),
        HolonomicWaypoint(PositionConstraint(1.57, { 0,  4})),
        HolonomicWaypoint(PositionConstraint(1.57, {-4,  0})),
        HolonomicWaypoint(PositionConstraint(1.57, { 0, -4})),
        HolonomicWaypoint(PositionConstraint(1.57, { 4,  0}), PositionConstraint(), HolonomicVelocityConstraint(0.0, {0.0, 0.0}))
    }, Obstacle(0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}})));

    // HolonomicPath holonomicPath(HolonomicPath({
    //     HolonomicWaypoint(0, 0, 0, 0, 0, 0, true, true, true,  true,  true,  true,  true,    0, {}, {}),
    //     HolonomicWaypoint(4, 0, 0, 0, 0, 0, true, true, true,  true,  true,  true,  true,   50, {}, {})
    // }));


    // benchmarks: before converting to std::vector: 2:08 -> 0:26 :)
    // std::cout << "Drivetrain:\n" << drive << "\n"
    //         << "\nPath:\n" << path << std::endl;

    Trajectory trajectory = OptimalTrajectoryGenerator::Generate(swerveDrivetrain, holonomicPath);

    std::cout << "\nTrajectory:\n\n" << trajectory;
    std::cout << std::endl;
}