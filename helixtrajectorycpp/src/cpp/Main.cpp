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
    // using namespace casadi;
    // Opti opti;
    // MX x = opti.variable(3);
    // MX y = x / 2;
    // MX z = fmin(x, 2.0);
    // opti.minimize(x(0)*x(0) + 2*x(0) - 1 + x(1)*x(1) + 2*x(1) + x(2)*x(2) - 5*x(2));
    // opti.solver("ipopt");
    // OptiSol solution = opti.solve();
    // std::cout << solution.value(x);
    // std::cout << solution.value(y);
    using namespace helixtrajectory;
    SwerveDrivetrain drive(45, 6,
            {{+0.6, +0.6, 0.04, 70, 2},
             {+0.6, -0.6, 0.04, 70, 2},
             {-0.6, +0.6, 0.04, 70, 2},
             {-0.6, -0.6, 0.04, 70, 2}},
            Obstacle(0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}}));

    HolonomicPath path(HolonomicPath({
        HolonomicWaypoint( 4,  0,    0, 0, 0, 0, true, true, true,  true,  true,  true,  true,   0, {}, {}),
        HolonomicWaypoint( 0,  4, 1.57, 0, 0, 0, true, true, true, false, false, false, false,   5, {}, {}),
        HolonomicWaypoint(-4,  0,    0, 0, 0, 0, true, true, true, false, false, false, false,   5, {}, {}),
        HolonomicWaypoint( 0, -4, 3.14, 0, 0, 0, true, true, true, false, false, false, false,   5, {}, {}),
        HolonomicWaypoint( 4,  0, 4.71, 0, 0, 0, true, true, true,  true,  true,  true,  true,   5, {}, {})
    }));
    // HolonomicPath path({
    //     HolonomicWaypoint(-4,  0,     0, 0, 0, 0, true, true, true,  true,  true,  true,  true,  0, {}, {}),
    //     HolonomicWaypoint( 0,  0,  1.57, 0, 0, 0, true, true, true, false, false, false, false, 4, {}, {}),
    //     HolonomicWaypoint( 4,  0,     0, 0, 0, 0, true, true, true,  true,  true,  true,  true, 4, {}, {})
    // });
    std::vector<Obstacle> obstacles;// = {Obstacle(0.2, {{0, 0}})};
    // std::cout << "Drivetrain:\n" << drive << "\n"
    //         << "\nPath:\n" << path << std::endl;

    
    HolonomicTrajectory trajectory = OptimalTrajectoryGenerator::Generate(drive, path);
    std::cout << "\nTrajectory:\n\n" << trajectory;
    std::cout << std::endl;
}