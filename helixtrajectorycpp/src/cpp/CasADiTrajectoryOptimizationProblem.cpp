#include "DebugOptions.h"

#include "CasADiTrajectoryOptimizationProblem.h"

#include <array>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include <casadi/casadi.hpp>

#include "Drivetrain.h"
#include "HolonomicPath.h"
#include "HolonomicTrajectory.h"
#include "Obstacle.h"
#include "Path.h"

namespace helixtrajectory {

    const casadi::Slice CasADiTrajectoryOptimizationProblem::ALL = casadi::Slice();

    CasADiTrajectoryOptimizationProblem::CasADiTrajectoryOptimizationProblem(const Drivetrain& drivetrain, const Path& path)
            : drivetrain(drivetrain), path(path),
            waypointCount(path.Length()), trajectorySegmentCount(waypointCount - 1),
            controlIntervalTotal(path.ControlIntervalTotal()),
            sampleTotal(controlIntervalTotal + 1), opti(),
            dt(), x(), y(), theta(),
            dtSegments(), xSegments(), ySegments(), thetaSegments() {

        dt.reserve(controlIntervalTotal);
        x.reserve(sampleTotal);
        y.reserve(sampleTotal);
        theta.reserve(sampleTotal);

        for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
            x.push_back(opti.variable());
            y.push_back(opti.variable());
            theta.push_back(opti.variable());
        }

        casadi::MX totalT = 0;
        size_t intervalIndex = 0;
        for (size_t trajectorySegmentIndex = 0; trajectorySegmentIndex < trajectorySegmentCount; trajectorySegmentIndex++) {
            casadi::MX segmentDt = opti.variable();
            size_t segmentControlIntervalCount = path.GetWaypoint(trajectorySegmentIndex + 1).controlIntervalCount;
            for (size_t segmentIntervalIndex = 0; segmentIntervalIndex < segmentControlIntervalCount; segmentIntervalIndex++) {
                dt.push_back(segmentDt);
                intervalIndex++;
            }
            totalT += segmentControlIntervalCount * segmentDt;
            opti.subject_to(segmentDt >= 0);
            opti.set_initial(segmentDt, 5.0 / segmentControlIntervalCount);
        }
#ifdef DEBUG_OUTPUT
        std::cout << "Applied Time Constraints" << std::endl;
#endif
        opti.minimize(totalT);
#ifdef DEBUG_OUTPUT
        std::cout << "Set Optimization Objective" << std::endl;
#endif

        dtSegments.reserve(trajectorySegmentCount);
        xSegments.reserve(waypointCount);
        ySegments.reserve(waypointCount);
        thetaSegments.reserve(waypointCount);
        xSegments.push_back({x[0]});
        ySegments.push_back({y[0]});
        thetaSegments.push_back({theta[0]});
        size_t sampleIndex = 1;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t controlIntervalCount = CasADiTrajectoryOptimizationProblem::path.GetWaypoint(waypointIndex)
                    .controlIntervalCount;
            std::vector<casadi::MX> dtSegment;
            std::vector<casadi::MX> xSegment;
            std::vector<casadi::MX> ySegment;
            std::vector<casadi::MX> thetaSegment;
            dtSegment.reserve(controlIntervalCount);
            xSegment.reserve(controlIntervalCount);
            ySegment.reserve(controlIntervalCount);
            thetaSegment.reserve(controlIntervalCount);
            for (size_t segmentSampleIndex = 0; segmentSampleIndex < controlIntervalCount; segmentSampleIndex++) {
                dtSegment.push_back(dt[(sampleIndex - 1) + segmentSampleIndex]);
                xSegment.push_back(x[sampleIndex + segmentSampleIndex]);
                ySegment.push_back(y[sampleIndex + segmentSampleIndex]);
                thetaSegment.push_back(theta[sampleIndex + segmentSampleIndex]);
            }
            dtSegments.push_back(dtSegment);
            xSegments.push_back(xSegment);
            ySegments.push_back(ySegment);
            thetaSegments.push_back(thetaSegment);
            sampleIndex += controlIntervalCount;
        }

        ApplyWaypointConstraints(opti, xSegments, ySegments, thetaSegments, CasADiTrajectoryOptimizationProblem::path);
#ifdef DEBUG_OUTPUT
        std::cout << "Applied Path constraints" << std::endl;
#endif

        ApplyObstacleConstraints(opti, xSegments, ySegments, thetaSegments,
                CasADiTrajectoryOptimizationProblem::drivetrain, CasADiTrajectoryOptimizationProblem::path);
#ifdef DEBUG_OUTPUT
        std::cout << "Applied Obstacle constraints" << std::endl;
#endif

        ApplyInitialGuessX(opti, x, y, theta, GenerateInitialGuessX(CasADiTrajectoryOptimizationProblem::path));
        // opti.set_initial(X, GenerateInitialGuessX(CasADiTrajectoryOptimizationProblem::path));
#ifdef DEBUG_OUTPUT
        std::cout << "Set Initial Trajectory" << std::endl;
#endif
    }

    void CasADiTrajectoryOptimizationProblem::ApplyWaypointConstraints(casadi::Opti& opti,
            const std::vector<std::vector<casadi::MX>>& xSegments, const std::vector<std::vector<casadi::MX>>& ySegments,
            const std::vector<std::vector<casadi::MX>>& thetaSegments, const Path& path) {
        for (int waypointIndex = 0; waypointIndex < path.Length(); waypointIndex++) {
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            size_t sampleIndex = xSegments[waypointIndex].size() - 1;
            if (waypoint.xConstrained) {
                opti.subject_to(xSegments[waypointIndex][sampleIndex] == waypoint.x);
            }
            if (waypoint.yConstrained) {
                opti.subject_to(ySegments[waypointIndex][sampleIndex] == waypoint.y);
            }
            if (waypoint.headingConstrained) {
                // opti.subject_to(fmod(thetaSegments[waypointIndex](-1) - waypoint.heading, 2 * M_PI) == 0.0);
                opti.subject_to(thetaSegments[waypointIndex][sampleIndex] == waypoint.heading);
            }
        }
    }

    const CasADiTrajectoryOptimizationProblem::BumperCornerPosition CasADiTrajectoryOptimizationProblem::SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, const ObstaclePoint& bumperCorner) {
        BumperCornerPosition position{};
        if (bumperCorner.x == 0.0 && bumperCorner.y == 0.0) {
            position.x = x;
            position.y = y;
        } else {
            double cornerDiagonal = hypot(bumperCorner.x, bumperCorner.y);
            double cornerAngle = atan2(bumperCorner.y, bumperCorner.x);
            position.x = x + cornerDiagonal * cos(cornerAngle + theta);
            position.y = y + cornerDiagonal * sin(cornerAngle + theta);
        }
        return position;
    }

    // https://www.desmos.com/calculator/cqmc1tjtsv
    template<typename LineNumberType, typename PointNumberType>
    casadi::MX linePointDist(LineNumberType lineStartX, LineNumberType lineStartY, LineNumberType lineEndX, LineNumberType lineEndY,
            PointNumberType pointX, PointNumberType pointY) {
        casadi::MX lX = lineEndX - lineStartX;
        casadi::MX lY = lineEndY - lineStartY;
        casadi::MX vX = pointX - lineStartX;
        casadi::MX vY = pointY - lineStartY;
        casadi::MX dot = vX * lX + vY * lY;
        casadi::MX lNormSquared = lX * lX + lY * lY;
        casadi::MX t = dot / lNormSquared;
        casadi::MX tBounded = fmax(fmin(t, 1), 0);
        casadi::MX iX = (1 - tBounded) * lineStartX + tBounded * lineEndX;
        casadi::MX iY = (1 - tBounded) * lineStartY + tBounded * lineEndY;
        casadi::MX distSquared = (iX - pointX) * (iX - pointX) + (iY - pointY) * (iY - pointY);
        return distSquared;
    }

    void CasADiTrajectoryOptimizationProblem::ApplyObstacleConstraint(casadi::Opti& opti, const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, const Obstacle& bumpers, const Obstacle& obstacle) {
        double distSquared = bumpers.safetyDistance + obstacle.safetyDistance;
        distSquared = distSquared * distSquared;
        size_t bumperCornerCount = bumpers.points.size();
        size_t obstacleCornerCount = obstacle.points.size();
        if (bumperCornerCount == 1 && obstacleCornerCount == 1) {
            // if the bumpers are only one point and the obstacle is also only one point, 
            const ObstaclePoint& bumperCorner = bumpers.points[0];
            const ObstaclePoint& obstaclePoint = obstacle.points[0];
            const BumperCornerPosition bumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumperCorner);
            casadi::MX deltaX = obstaclePoint.x - bumperCornerPosition.x;
            casadi::MX deltaY = obstaclePoint.y - bumperCornerPosition.y;
            casadi::MX pointDistSquared = deltaX * deltaX + deltaY * deltaY;
            opti.subject_to(pointDistSquared >= distSquared);
        } else {
            for (size_t bumperCornerIndex = 0; bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
                // std::cout << "About to solve bumper positions" << std::endl;
                const BumperCornerPosition startBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerIndex]);
                const BumperCornerPosition endBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerIndex + 1]);
                // std::cout << "solved bumper positions" << std::endl;
                for (const ObstaclePoint& obstaclePoint : obstacle.points) {
                    // std::cout << "Finding line point dist" << std::endl;
                    casadi::MX dist = linePointDist(startBumperCornerPosition.x, startBumperCornerPosition.y,
                            endBumperCornerPosition.x, endBumperCornerPosition.y, obstaclePoint.x, obstaclePoint.y);
                    // std::cout << "Line point dist found" << std::endl;
                    opti.subject_to(dist >= distSquared);
                    // std::cout << "Constrained line point dist" << std::endl;
                }
            }
            if (bumperCornerCount >= 3) { // must have at least three points to make a closed polygon
                const BumperCornerPosition startBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerCount - 1]);
                const BumperCornerPosition endBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[0]);
                for (const ObstaclePoint& obstaclePoint : obstacle.points) {
                    casadi::MX dist = linePointDist(startBumperCornerPosition.x, startBumperCornerPosition.y,
                            endBumperCornerPosition.x, endBumperCornerPosition.y, obstaclePoint.x, obstaclePoint.y);
                    opti.subject_to(dist >= distSquared);
                }
            }

            for (size_t obstacleCornerIndex = 0; obstacleCornerIndex < obstacleCornerCount - 1; obstacleCornerIndex++) {
                double startObstacleCornerX = obstacle.points[obstacleCornerIndex].x;
                double startObstacleCornerY = obstacle.points[obstacleCornerIndex].y;
                double endObstacleCornerX = obstacle.points[obstacleCornerIndex + 1].x;
                double endObstacleCornerY = obstacle.points[obstacleCornerIndex + 1].y;
                for (const ObstaclePoint& bumperCorner : bumpers.points) {
                    const BumperCornerPosition bumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumperCorner);
                    casadi::MX dist = linePointDist(startObstacleCornerX, startObstacleCornerY,
                            endObstacleCornerX, endObstacleCornerY, bumperCornerPosition.x, bumperCornerPosition.y);
                    opti.subject_to(dist >= bumpers.safetyDistance + obstacle.safetyDistance);
                }
            }
            if (obstacleCornerCount >= 3) {
                double startObstacleCornerX = obstacle.points[obstacleCornerCount - 1].x;
                double startObstacleCornerY = obstacle.points[obstacleCornerCount - 1].y;
                double endObstacleCornerX = obstacle.points[0].x;
                double endObstacleCornerY = obstacle.points[0].y;
                for (const ObstaclePoint& bumperCorner : bumpers.points) {
                    const BumperCornerPosition bumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumperCorner);
                    casadi::MX dist = linePointDist(startObstacleCornerX, startObstacleCornerY,
                            endObstacleCornerX, endObstacleCornerY, bumperCornerPosition.x, bumperCornerPosition.y);
                    opti.subject_to(dist >= bumpers.safetyDistance + obstacle.safetyDistance);
                }
            }
        }
    }

    void CasADiTrajectoryOptimizationProblem::ApplyObstacleConstraints(casadi::Opti& opti, const std::vector<std::vector<casadi::MX>>& xSegments,
            const std::vector<std::vector<casadi::MX>>& ySegments, const std::vector<std::vector<casadi::MX>>& thetaSegments,
            const Drivetrain& drivetrain, const Path& path) {
        const Obstacle& bumpers = drivetrain.bumpers;
        std::vector<const Obstacle*> allSegmentsObstacles;
        for (size_t waypointIndex = 0; waypointIndex < path.Length(); waypointIndex++) {
            for (const Obstacle& obstacle : path.GetWaypoint(waypointIndex).obstacles) {
                if (obstacle.applyToAllSegments) {
                    allSegmentsObstacles.push_back(&obstacle);
                }
            }
        }
        for (size_t waypointIndex = 0; waypointIndex < path.Length(); waypointIndex++) {
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            const std::vector<casadi::MX>& xSegment = xSegments[waypointIndex];
            const std::vector<casadi::MX>& ySegment = ySegments[waypointIndex];
            const std::vector<casadi::MX>& thetaSegment = thetaSegments[waypointIndex];
            size_t segmentSampleCount = xSegment.size();
            for (size_t sampleIndex = 0; sampleIndex < segmentSampleCount; sampleIndex++) {
                for (const Obstacle* obstacle : allSegmentsObstacles) {
                    ApplyObstacleConstraint(opti, xSegment[sampleIndex], ySegment[sampleIndex], thetaSegment[sampleIndex], bumpers, *obstacle);
                }
                for (const Obstacle& obstacle : waypoint.obstacles) {
                    ApplyObstacleConstraint(opti, xSegment[sampleIndex], ySegment[sampleIndex], thetaSegment[sampleIndex], bumpers, obstacle);
                }
            }
        }
    }

    void linspace(std::vector<double>& arry, size_t startIndex, size_t endIndex, double startValue, double endValue) {
        size_t segmentCount = endIndex - startIndex;
        double delta = (endValue - startValue) / segmentCount;
        for (int index = 0; index < segmentCount; index++) {
            // arry[startIndex + index] = startValue + index * delta;
            arry.push_back(startValue + index * delta);
        }
    }

    const CasADiTrajectoryOptimizationProblem::InitialGuessX CasADiTrajectoryOptimizationProblem::GenerateInitialGuessX(const Path& path) {
        size_t waypointCount = path.Length();
        size_t controlIntervalTotal = path.ControlIntervalTotal();
        size_t sampleTotal = controlIntervalTotal + 1;
        InitialGuessX initialGuessX{{}, {}, {}};
        initialGuessX.x.reserve(sampleTotal);
        initialGuessX.y.reserve(sampleTotal);
        initialGuessX.theta.reserve(sampleTotal);
        size_t sampleIndex = path.GetWaypoint(0).controlIntervalCount;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            const Waypoint& previousWaypoint = path.GetWaypoint(waypointIndex - 1);
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            size_t intervalCount = waypoint.controlIntervalCount;
            size_t guessPointCount = waypoint.initialGuessPoints.size();
            size_t previousWaypointSampleIndex = sampleIndex;
            size_t waypointSampleIndex = previousWaypointSampleIndex + intervalCount;
            if (guessPointCount == 0) {
                linspace(initialGuessX.x, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.x, waypoint.x);
                linspace(initialGuessX.y, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.y, waypoint.y);
                linspace(initialGuessX.theta, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.heading, waypoint.heading);
            } else {
                size_t guessSegmentIntervalCount = intervalCount / (guessPointCount + 1);
                linspace(initialGuessX.x, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.x, waypoint.initialGuessPoints[0].x);
                linspace(initialGuessX.y, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.y, waypoint.initialGuessPoints[0].y);
                linspace(initialGuessX.theta, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.heading, waypoint.initialGuessPoints[0].heading);
                size_t firstGuessPointSampleIndex = previousWaypointSampleIndex + guessSegmentIntervalCount;
                for (int guessPointIndex = 1; guessPointIndex < guessPointCount; guessPointIndex++) {
                    size_t previousGuessPointSampleIndex = firstGuessPointSampleIndex + (guessPointIndex - 1) * guessSegmentIntervalCount;
                    size_t guessPointSampleIndex = firstGuessPointSampleIndex + (guessPointIndex) * guessSegmentIntervalCount;
                    linspace(initialGuessX.x, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].x, waypoint.initialGuessPoints[guessPointIndex].x);
                    linspace(initialGuessX.y, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].y, waypoint.initialGuessPoints[guessPointIndex].y);
                    linspace(initialGuessX.theta, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].heading, waypoint.initialGuessPoints[guessPointIndex].heading);
                }
                size_t finalGuessPointSampleIndex = previousWaypointSampleIndex + guessPointCount * guessSegmentIntervalCount;
                linspace(initialGuessX.x, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].x, waypoint.x);
                linspace(initialGuessX.y, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].y, waypoint.y);
                linspace(initialGuessX.theta, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].heading, waypoint.heading);
            }
            sampleIndex += intervalCount;
        }
        initialGuessX.x.push_back(path.GetWaypoint(waypointCount - 1).x);
        initialGuessX.y.push_back(path.GetWaypoint(waypointCount - 1).y);
        initialGuessX.theta.push_back(path.GetWaypoint(waypointCount - 1).heading);

        return initialGuessX;
    }

    void CasADiTrajectoryOptimizationProblem::ApplyInitialGuessX(casadi::Opti& opti, const std::vector<casadi::MX>& x,
            const std::vector<casadi::MX>& y, const std::vector<casadi::MX>& theta,
            const InitialGuessX& initialGuessX) {

        std::cout << "\n\nInitial Guess:\n";
        size_t sampleTotal = x.size();
        for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
            std::cout << initialGuessX.x[sampleIndex] << ", " << initialGuessX.y[sampleIndex] << ", " << initialGuessX.theta[sampleIndex] << "\n";
            opti.set_initial(x[sampleIndex], initialGuessX.x[sampleIndex]);
            opti.set_initial(y[sampleIndex], initialGuessX.y[sampleIndex]);
            opti.set_initial(theta[sampleIndex], initialGuessX.theta[sampleIndex]);
        }
        std::cout << std::endl;
    }
}