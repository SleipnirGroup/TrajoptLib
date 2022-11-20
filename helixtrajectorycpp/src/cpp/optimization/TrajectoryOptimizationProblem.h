#pragma once

#include <vector>

#include "constraint/Constraint.h"
#include "drivetrain/Drivetrain.h"
#include "obstacle/Obstacle.h"
#include "path/Path.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"

namespace helixtrajectory {

/**
 * @brief This class is the superclass for all trajectory generators. It contains the common
 * functionality of all optimizers: waypoint position constraints and obstacle avoidance.
 */
template<typename Opti>
class TrajectoryOptimizationProblem {
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
    Opti opti;

    /**
     * @brief an abstract expression type representing a scalar expression
     */
    using Expression = typename Opti::Expression;

    /**
     * @brief The 1 x (controlIntervalTotal) vector of the time differentials between sample points.
     * The nth entry in this vector is the duration of the nth interval in this trajectory.
     */
    std::vector<Expression> dt;

    /**
     * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's x-coordinate per trajectory sample point
     */
    std::vector<Expression> x;
    /**
     * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's y-coordinate per trajectory sample point
     */
    std::vector<Expression> y;
    /**
     * @brief the 1 x (controlIntervalTotal + 1) vector of the robot's heading per trajectory sample point
     */
    std::vector<Expression> theta;
    
    /**
     * @brief the entries of the dt vector, separated into individual trajectory segments
     */
    std::vector<std::vector<Expression>> dtSegments;

    /**
     * @brief the entries of the x vector, separated into individual trajectory segments
     */
    std::vector<std::vector<Expression>> xSegments;
    /**
     * @brief the entries of the y vector, separated into individual trajectory segments
     */
    std::vector<std::vector<Expression>> ySegments;
    /**
     * @brief the entries of the theta vector, separated into individual trajectory segments
     */
    std::vector<std::vector<Expression>> thetaSegments;

    /**
     * @brief Construct a new CasADi Trajectory Optimization Problem from a drivetrain, path.
     * 
     * @param drivetrain the drivetrain
     * @param path the path
     */
    TrajectoryOptimizationProblem(const Drivetrain& drivetrain, const Path& path);

    static void ApplyIntervalSet1dConstraint(Opti& opti, const Expression& scalar, const IntervalSet1d& set1d);
    static void ApplySet2dConstraint(Opti& opti, const Expression& vectorX, const Expression& vectorY, const Set2d& set2d);

private:
    struct BumperCornerPosition {
        Expression x;
        Expression y;
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
    static const BumperCornerPosition SolveBumperCornerPosition(const Expression& x, const Expression& y,
            const Expression& theta, const ObstaclePoint& bumperCorner);

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
    static void ApplyObstacleConstraint(Opti& opti, const Expression& x, const Expression& y,
            const Expression& theta, const Obstacle& bumpers, const Obstacle& obstacle);

    static void ApplyConstraint(Opti& opti, const Expression& x, const Expression& y,
            const Expression& theta, const Obstacle& bumpers, const Constraint& constraint);

    static void ApplyConstraints(Opti& opti, const Expression& x, const Expression& y,
            const Expression& theta, const Obstacle& bumpers, const std::vector<Constraint>& constraint);

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
    static void ApplyPathConstraints(Opti& opti,
            const std::vector<std::vector<Expression>>& xSegments,
            const std::vector<std::vector<Expression>>& ySegments,
            const std::vector<std::vector<Expression>>& thetaSegments,
            const Path& path);

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

    static void ApplyInitialGuessX(Opti& opti,
            const std::vector<Expression>& x,
            const std::vector<Expression>& y,
            const std::vector<Expression>& theta,
            const InitialGuessX& initialGuessX);

public:
    /**
     * @brief Destroy the Trajectory Optimization Problem object
     */
    virtual ~TrajectoryOptimizationProblem() = default;
};
}