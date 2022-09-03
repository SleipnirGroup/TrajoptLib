#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicPath.h"
#include "HolonomicWaypoint.h"
#include "Obstacle.h"
#include "OptimalTrajectoryGenerator.h"
#include "SwerveDrivetrain.h"
#include "SwerveModule.h"

int main() {
    using namespace helixtrajectory;
    SwerveDrivetrain swerveDrivetrain(45, 6,
            {SwerveModule(+0.6, +0.6, 0.04, 70, 2),
             SwerveModule(+0.6, -0.6, 0.04, 70, 2),
             SwerveModule(-0.6, +0.6, 0.04, 70, 2),
             SwerveModule(-0.6, -0.6, 0.04, 70, 2)},
            Obstacle(0, true, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}}));

    HolonomicPath holonomicPath(HolonomicPath({
        HolonomicWaypoint( 4,  0,    0, 0, 0, 0, true, true, true,  true,  true,  true,  true,    0, {}, {}),
        HolonomicWaypoint( 0,  4, 1.57, 0, 0, 0, true, true, true, false, false, false, false,   30, {}, {}),
        HolonomicWaypoint(-4,  0,    0, 0, 0, 0, true, true, true, false, false, false, false,   30, {}, {}),
        HolonomicWaypoint( 0, -4, 3.14, 0, 0, 0, true, true, true, false, false, false, false,   30, {}, {}),
        HolonomicWaypoint( 4,  0, 4.71, 0, 0, 0, true, true, true,  true,  true,  true,  true,   30, {}, {})
    }));

    // std::cout << "Drivetrain:\n" << drive << "\n"
    //         << "\nPath:\n" << path << std::endl;

    HolonomicTrajectory holonomicTrajectory = OptimalTrajectoryGenerator::Generate(swerveDrivetrain, holonomicPath);

    std::cout << "\nTrajectory:\n\n" << holonomicTrajectory;
    std::cout << std::endl;
}