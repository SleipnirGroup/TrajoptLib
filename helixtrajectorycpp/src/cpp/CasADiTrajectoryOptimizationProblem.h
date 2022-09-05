#pragma once

#include <array>
#include <vector>

#include <casadi/casadi.hpp>

#include "Drivetrain.h"
#include "HolonomicDrivetrain.h"
#include "Obstacle.h"
#include "Path.h"

namespace helixtrajectory {

    /**
     * @brief This class is the superclass for all trajectory generators. It contains the common
     * functionality of all optimizers: waypoint position constraints and obstacle avoidance.
     */
    class CasADiTrajectoryOptimizationProblem {
    protected:
        /**
         * @brief the drivetrain
         */
        const Drivetrain& drivetrain;
        /**
         * @brief the path
         */
        const Path& path;

        /**
         * @brief the number of waypoints in the path
         */
        const size_t waypointCount;
        /**
         * @brief the number of trajectory segments in the trajectory
         */
        const size_t trajectorySegmentCount;
        /**
         * @brief the total number of control intervals in the trajectory
         */
        const size_t controlIntervalTotal;
        /**
         * @brief the total number of sample points in the trajectory (controlIntervalTotal + 1)
         */
        const size_t sampleTotal;

        /**
         * @brief the optimizer
         */
        casadi::Opti opti;

        /**
         * @brief The 1 x (controlIntervalTotal) vector of the time differentials between sample points.
         * The nth entry in this vector is the duration of the nth interval in this trajectory.
         */
        std::vector<casadi::MX> dt;

        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's x-coordinate per trajectory sample point
         */
        std::vector<casadi::MX> x;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's y-coordinate per trajectory sample point
         */
        std::vector<casadi::MX> y;
        /**
         * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's heading per trajectory sample point
         */
        std::vector<casadi::MX> theta;
        
        /**
         * @brief the entries of the dt vector, separated into individual trajectory segments
         */
        std::vector<std::vector<casadi::MX>> dtSegments;

        /**
         * @brief the entries of the x vector, separated into individual trajectory segments
         */
        std::vector<std::vector<casadi::MX>> xSegments;
        /**
         * @brief the entries of the y vector, separated into individual trajectory segments
         */
        std::vector<std::vector<casadi::MX>> ySegments;
        /**
         * @brief the entries of the theta vector, separated into individual trajectory segments
         */
        std::vector<std::vector<casadi::MX>> thetaSegments;

        /**
         * @brief Construct a new CasADi Trajectory Optimization Problem from a drivetrain, path.
         * 
         * @param drivetrain the drivetrain
         * @param path the path
         */
        CasADiTrajectoryOptimizationProblem(const Drivetrain& drivetrain, const Path& path);

        /**
         * @brief slice all rows/columns of a matrix
         */
        static const casadi::Slice ALL;

    private:
        /**
         * @brief Apply the constraints that force the robot's motion to comply
         * with the list of waypoints provided. This function only applies constraints
         * on position and heading while velocity constraints are left up to different
         * drivetrain optimization problems.
         * 
         * @param opti the current optimizer
         * @param xSegments the x-coordinate of the robot for each sample point, divided into segments
         * @param ySegments the y-coordinate of the robot for each sample point, divided into segments
         * @param thetaSegments the heading of the robot for each sample point, divided into segments
         * @param path the path containing the waypoints to constrain
         */
        static void ApplyWaypointConstraints(casadi::Opti& opti,
                const std::vector<std::vector<casadi::MX>>& xSegments, const std::vector<std::vector<casadi::MX>>& ySegments,
                const std::vector<std::vector<casadi::MX>>& thetaSegments, const Path& path);

        struct BumperCornerPosition {
            casadi::MX x;
            casadi::MX y;
        };

        /**
         * @brief Get an expression for the position of a bumper corner relative
         * to the field coordinate system, given the robot's x-coordinate, y-coordinate,
         * and heading. The first row of the resulting matrix contains the x-coordinate,
         * and the second row contains the y-coordinate.
         * 
         * @param x the instantaneous heading of the robot (scalar)
         * @param y the instantaneous heading of the robot (scalar)
         * @param theta the instantaneous heading of the robot (scalar)
         * @param bumperCorner the bumper corner to find the position for
         * @return the bumper corner 2 x 1 position vector
         */
        static const BumperCornerPosition SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
                const casadi::MX& theta, const ObstaclePoint& bumperCorner);

        /**
         * @brief Apply obstacle constraints for a single obstacle at a single
         * sample point in the trajectory. If the robot's bumpers and an obstacle
         * are made from a single point, a minimum distance constraint is applied.
         * Otherwise, constraints that prevent a point on the bumpers from getting
         * too close to an obstacle line segment and prevent a line segment on the
         * bumpers from getting too close to an obstacle point are created.
         * 
         * @param opti the current optimizer
         * @param x the instantaneous heading of the robot (scalar)
         * @param y the instantaneous heading of the robot (scalar)
         * @param theta the instantaneous heading of the robot (scalar)
         * @param bumpers the obstacle that represents the robot's bumpers
         * @param obstacle the obstacle to apply the constraint for
         */
        static void ApplyObstacleConstraint(casadi::Opti& opti, const casadi::MX& x, const casadi::MX& y,
                const casadi::MX& theta, const Obstacle& bumpers, const Obstacle& obstacle);

        /**
         * @brief Apply constraints that prevent the robot from getting too close
         * to obstacles. Constraints are applied to the specified segments
         * of the path or the entire path if that is specified.
         * 
         * @param opti the current optimizer
         * @param xSegments the x-coordinate of the robot for each sample point, divided into segments
         * @param ySegments the y-coordinate of the robot for each sample point, divided into segments
         * @param thetaSegments the heading of the robot for each sample point, divided into segments
         * @param drivetrain the drivetrain containing the bumpers to apply constraints for
         * @param path the path containing the obstacles to apply constraints for
         */
        static void ApplyObstacleConstraints(casadi::Opti& opti, const std::vector<std::vector<casadi::MX>>& xSegments,
                const std::vector<std::vector<casadi::MX>>& ySegments, const std::vector<std::vector<casadi::MX>>& thetaSegments,
                const Drivetrain& drivetrain, const Path& path);

        struct InitialGuessX {
            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> theta;
        };

        /**
         * @brief Generate an initial guess of the position state of the robot throughout
         * the trajectory. The initial guess is a linear interpolation between the waypoints
         * and initial guess points.
         * 
         * @param path the path to calucluate the linear initial guess for
         * @return an matrix of augmented column vectors of position state (x, y, heading)
         */
        static const InitialGuessX GenerateInitialGuessX(const Path& path);

        static void ApplyInitialGuessX(casadi::Opti& opti, const std::vector<casadi::MX>& x,
                const std::vector<casadi::MX>& y, const std::vector<casadi::MX>& theta,
                const InitialGuessX& initialGuessX);

    public:
        /**
         * @brief Destroy the CasADi Trajectory Optimization Problem object
         */
        virtual ~CasADiTrajectoryOptimizationProblem() = default;

        virtual void PrintSolution(const casadi::OptiSol& solution) const = 0;
    };
}