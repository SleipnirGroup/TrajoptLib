#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>

#include <casadi/casadi.hpp>
#include <sleipnir/autodiff/Variable.hpp>
#include <sleipnir/optimization/OptimizationProblem.hpp>

#include "path/HolonomicPath.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "path/InitialGuessPoint.h"
#include "obstacle/Obstacle.h"
#include "OptimalTrajectoryGenerator.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "trajectory/HolonomicTrajectory.h"
#include "IncompatibleTrajectoryException.h"
#include "set/ConeSet2d.h"
#include "TestUtil.h"

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

    Obstacle bumpers(0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}});

    // HolonomicPath holonomicPath(HolonomicPath({
    //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 4,  0},  0.00)},     {VelocityHolonomicConstraint(RectangularSet2d{0, 0}), AngularVelocityConstraint(0.0)}, {}, {},   0, {InitialGuessPoint( 4,  0,  0.00)}),
    //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0,  4},  1.57)},     {},                                                                                    {}, {},  10, {InitialGuessPoint( 0,  4,  1.57)}),
    //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{-4,  0},  0.00)},     {},                                                                                    {}, {},  10, {InitialGuessPoint(-4,  0,  0.00)}),
    //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0, -4}, -1.57)},     {},                                                                                    {}, {},  10, {InitialGuessPoint( 0, -4, -1.57)}),
    //     HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 4,  0},  0.00)},     {VelocityHolonomicConstraint(RectangularSet2d{0, 0}), AngularVelocityConstraint(0.0)}, {}, {},  10, {InitialGuessPoint( 4,  0,  0.00)})},
    //     Obstacle(0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}})));

    // OBSTACLE TEST:
    const std::vector<InitialGuessPoint> guesses = {
        { 0.00,  0.00, 0.00},
        { 2.00,  1.50, 0.00},
        { 3.60,  0.00, -M_PI_2},
        { 2.00, -1.50, -M_PI},
        { 0.00,  0.00, -M_PI}
    };
    Obstacle cone(0.8, {{2.0, 0.0}});
    HolonomicPath holonomicPath({
        HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0,  0},  0.00)}, {VelocityHolonomicConstraint(RectangularSet2d{0, 0}), AngularVelocityConstraint(0.0)}, {                        }, {},   0, {InitialGuessPoint( 0,  0,   0.00)}),
        HolonomicWaypoint({PoseConstraint(RectangularSet2d{ 0,  0}, -M_PI)}, {VelocityHolonomicConstraint(RectangularSet2d{0, 0}), AngularVelocityConstraint(0.0)}, {ObstacleConstraint(cone)}, {}, 100,                             guesses)},
        bumpers);

    // const Obstacle initialBoundary = Obstacle(0.0, {{1.524, -1.524}, {1.524, -3.048}, {0.000, -3.048}, {0.000, -1.524}});
    // const Obstacle d5 = Obstacle(0.1, {{3.81, -3.048}});
    // const Obstacle b8 = Obstacle(0.1, {{6.096, -1.524}});
    // const Obstacle d10 = Obstacle(0.1, {{7.62, -3.048}});
    // const std::vector<InitialGuessPoint> guesses = {
    //     {4.32, -2.61, 0.0},
    //     {4.34, -3.86, 0.0},
    //     {3.00, -3.40, 0.0},
    //     {3.31, -2.48, 0.0},
    //     {6.74, -2.29, 0.0},
    //     {6.80, -0.77, 0.0},
    //     {5.30, -0.76, 0.0},
    //     {5.20, -2.31, 0.0},
    //     {7.53, -4.01, 0.0},
    //     {8.51, -3.55, 0.0},
    //     {8.39, -2.45, 0.0},
    //     {0.762, -2.286, 0.0}
    // };
    // constexpr double inf = std::numeric_limits<double>::infinity();
    // const std::vector<HolonomicConstraint> zeroVelocityConstraints = {VelocityHolonomicConstraint(RectangularSet2d{0, 0}), AngularVelocityConstraint(0.0)};
    // HolonomicPath holonomicPath(HolonomicPath({
    //     HolonomicWaypoint({PoseConstraint(RectangularSet2d(0.762, -2.286), 0.0)}, zeroVelocityConstraints, {}, {}, 0, {InitialGuessPoint(0.762, -2.286, 0.00)}),
    //     HolonomicWaypoint({TranslationConstraint(RectangularSet2d(0.762, -2.286))}, {}, {ObstacleConstraint(d5), ObstacleConstraint(b8), ObstacleConstraint(d10)}, {}, 200, guesses)},
    //     Obstacle(0, {{+0.5, +0.5}, {-0.5, +0.5}, {-0.5, -0.5}, {+0.5, -0.5}})));



    // benchmarks: before converting to std::vector: 2:08 -> 0:26 :)
    // std::cout << "Drivetrain:\n" << drive << "\n"
    //         << "\nPath:\n" << path << std::endl;

    HolonomicTrajectory trajectory = OptimalTrajectoryGenerator::Generate(swerveDrivetrain, holonomicPath);

    // std::cout << "\nTrajectory:\n\n" << trajectory << std::endl;

    std::cout << std::endl;


    // AngularVelocityConstraint constraint = IntervalSet1d(1, 2);
    // try {
    //     constraint.CheckAngularVelocity(-1);
    // } catch (const IncompatibleTrajectoryException& exception) {
    //     std::cout << exception.what();
    // }

    // ConeSet2d set2d = IntervalSet1d(0, 1.57/2);
    // try {
    //     set2d.CheckVector(1,0);
    //     set2d.CheckVector(1,0.5);
    //     set2d.CheckVector(0.5,1);
    // } catch (const IncompatibleTrajectoryException& exception) {
    //     std::cout << exception.what();
    // }

    // casadi::Opti opti;
    // opti.solver("ipopt");
    // auto x = opti.variable();
    // // opti.subject_to(x >= 0 && x <= 1);
    // opti.subject_to(x >= 0);
    // opti.subject_to(x <= 1);
    // opti.minimize(-2 * x * x);
    // auto sol = opti.solve();
    // std::cout << "Solver value: " << static_cast<double>(sol.value(x)) << std::endl;
    // ConeSet2d set(IntervalSet1d(0, 0.78));
    // std::array<bool, 2> carry = set.GetConstraints(2., -1.);
    // std::cout << std::boolalpha;
    // std::cout << carry[0] << ", " << carry[1] << std::endl;

    sleipnir::OptimizationProblem problem;
    sleipnir::VariableMatrix vari = problem.DecisionVariable();
    problem.SubjectTo(vari >= 3);
    problem.Minimize(vari * vari);
    // problem.SubjectTo(Max(vari, 1) == 0.0);
    problem.Solve();
    std::cout << "Variable is: " << vari.Value();

}