#include "DebugOptions.h"

#include "CasADiTrajectoryOptimizationProblem.h"

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
            dt(1, controlIntervalTotal), X(opti.variable(3, controlIntervalTotal + 1)),
            x(X(0, ALL)), y(X(1, ALL)), theta(X(2, ALL)),
            XSegments(), xSegments(), ySegments(), thetaSegments(),
            dtSegments() {

        casadi::MX totalT = 0;
        size_t intervalIndex = 0;
        for (size_t trajectorySegmentIndex = 0; trajectorySegmentIndex < trajectorySegmentCount; trajectorySegmentIndex++) {
            casadi::MX segmentDt = opti.variable();
            size_t segmentControlIntervalCount = path.GetWaypoint(trajectorySegmentIndex + 1).controlIntervalCount;
            for (size_t segmentIntervalIndex = 0; segmentIntervalIndex < segmentControlIntervalCount; segmentIntervalIndex++) {
                dt(intervalIndex) = segmentDt;
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
        XSegments.reserve(waypointCount);
        xSegments.reserve(waypointCount);
        ySegments.reserve(waypointCount);
        thetaSegments.reserve(waypointCount);
        XSegments.push_back(X(ALL, 0));
        xSegments.push_back(x(ALL, 0));
        ySegments.push_back(y(ALL, 0));
        thetaSegments.push_back(theta(ALL, 0));
        size_t sampleIndex = 1;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            size_t controlIntervalCount = CasADiTrajectoryOptimizationProblem::path.GetWaypoint(waypointIndex)
                    .controlIntervalCount;
            casadi::Slice dtSlice((int) (sampleIndex - 1), (int) (sampleIndex - 1 + controlIntervalCount));
            casadi::Slice XSlice((int) sampleIndex, (int) (sampleIndex + controlIntervalCount));
            dtSegments.push_back(dt(ALL, dtSlice));
            XSegments.push_back(X(ALL, XSlice));
            xSegments.push_back(x(ALL, XSlice));
            ySegments.push_back(y(ALL, XSlice));
            thetaSegments.push_back(theta(ALL, XSlice));
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

        opti.set_initial(X, GenerateInitialGuessX(CasADiTrajectoryOptimizationProblem::path));
#ifdef DEBUG_OUTPUT
        std::cout << "Set Initial Trajectory" << std::endl;
#endif
    }

    void CasADiTrajectoryOptimizationProblem::ApplyWaypointConstraints(casadi::Opti& opti,
                const std::vector<casadi::MX>& xSegments, const std::vector<casadi::MX>& ySegments,
                const std::vector<casadi::MX>& thetaSegments, const Path& path) {
        for (int waypointIndex = 0; waypointIndex < path.Length(); waypointIndex++) {
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            if (waypoint.xConstrained) {
                opti.subject_to(xSegments[waypointIndex](-1) == waypoint.x);
            }
            if (waypoint.yConstrained) {
                opti.subject_to(ySegments[waypointIndex](-1) == waypoint.y);
            }
            if (waypoint.headingConstrained) {
                // opti.subject_to(fmod(thetaSegments[waypointIndex](-1) - waypoint.heading, 2 * M_PI) == 0.0);
                opti.subject_to(thetaSegments[waypointIndex](-1) == waypoint.heading);
            }
        }
    }

    const casadi::MX CasADiTrajectoryOptimizationProblem::SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, const ObstaclePoint& bumperCorner) {
        casadi::MX position(2, 1);
        if (bumperCorner.x == 0.0 && bumperCorner.y == 0.0) {
            position(0) = x;
            position(1) = y;
        } else {
            double cornerDiagonal = hypot(bumperCorner.x, bumperCorner.y);
            double cornerAngle = atan2(bumperCorner.y, bumperCorner.x);
            position(0) = x + cornerDiagonal * cos(cornerAngle + theta);
            position(1) = y + cornerDiagonal * sin(cornerAngle + theta);
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
            casadi::MX bumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumperCorner);
            casadi::MX deltaX = obstaclePoint.x - bumperCornerPosition(0);
            casadi::MX deltaY = obstaclePoint.y - bumperCornerPosition(1);
            casadi::MX pointDistSquared = deltaX * deltaX + deltaY * deltaY;
            opti.subject_to(pointDistSquared >= distSquared);
        } else {
            for (size_t bumperCornerIndex = 0; bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
                // std::cout << "About to solve bumper positions" << std::endl;
                casadi::MX startBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerIndex]);
                casadi::MX endBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerIndex + 1]);
                // std::cout << "solved bumper positions" << std::endl;
                for (const ObstaclePoint& obstaclePoint : obstacle.points) {
                    // std::cout << "Finding line point dist" << std::endl;
                    casadi::MX dist = linePointDist(startBumperCornerPosition(0), startBumperCornerPosition(1),
                            endBumperCornerPosition(0), endBumperCornerPosition(1), obstaclePoint.x, obstaclePoint.y);
                    // std::cout << "Line point dist found" << std::endl;
                    opti.subject_to(dist >= distSquared);
                    // std::cout << "Constrained line point dist" << std::endl;
                }
            }
            if (bumperCornerCount >= 3) { // must have at least three points to make a closed polygon
                casadi::MX startBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerCount - 1]);
                casadi::MX endBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[0]);
                for (const ObstaclePoint& obstaclePoint : obstacle.points) {
                    casadi::MX dist = linePointDist(startBumperCornerPosition(0), startBumperCornerPosition(1),
                            endBumperCornerPosition(0), endBumperCornerPosition(1), obstaclePoint.x, obstaclePoint.y);
                    opti.subject_to(dist >= distSquared);
                }
            }

            for (size_t obstacleCornerIndex = 0; obstacleCornerIndex < obstacleCornerCount - 1; obstacleCornerIndex++) {
                double startObstacleCornerX = obstacle.points[obstacleCornerIndex].x;
                double startObstacleCornerY = obstacle.points[obstacleCornerIndex].y;
                double endObstacleCornerX = obstacle.points[obstacleCornerIndex + 1].x;
                double endObstacleCornerY = obstacle.points[obstacleCornerIndex + 1].y;
                for (const ObstaclePoint& bumperCorner : bumpers.points) {
                    casadi::MX bumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumperCorner);
                    casadi::MX dist = linePointDist(startObstacleCornerX, startObstacleCornerY,
                            endObstacleCornerX, endObstacleCornerY, bumperCornerPosition(0), bumperCornerPosition(1));
                    opti.subject_to(dist >= bumpers.safetyDistance + obstacle.safetyDistance);
                }
            }
            if (obstacleCornerCount >= 3) {
                double startObstacleCornerX = obstacle.points[obstacleCornerCount - 1].x;
                double startObstacleCornerY = obstacle.points[obstacleCornerCount - 1].y;
                double endObstacleCornerX = obstacle.points[0].x;
                double endObstacleCornerY = obstacle.points[0].y;
                for (const ObstaclePoint& bumperCorner : bumpers.points) {
                    casadi::MX bumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumperCorner);
                    casadi::MX dist = linePointDist(startObstacleCornerX, startObstacleCornerY,
                            endObstacleCornerX, endObstacleCornerY, bumperCornerPosition(0), bumperCornerPosition(1));
                    opti.subject_to(dist >= bumpers.safetyDistance + obstacle.safetyDistance);
                }
            }
        }
    }

    void CasADiTrajectoryOptimizationProblem::ApplyObstacleConstraints(casadi::Opti& opti, const std::vector<casadi::MX>& xSegments,
            const std::vector<casadi::MX>& ySegments, const std::vector<casadi::MX>& thetaSegments,
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
            const casadi::MX& xSegment = xSegments[waypointIndex];
            const casadi::MX& ySegment = ySegments[waypointIndex];
            const casadi::MX& thetaSegment = thetaSegments[waypointIndex];
            size_t segmentSampleCount = xSegment.columns();
            for (size_t sampleIndex = 0; sampleIndex < segmentSampleCount; sampleIndex++) {
                for (const Obstacle* obstacle : allSegmentsObstacles) {
                    ApplyObstacleConstraint(opti, xSegment(sampleIndex), ySegment(sampleIndex), thetaSegment(sampleIndex), bumpers, *obstacle);
                }
                for (const Obstacle& obstacle : waypoint.obstacles) {
                    ApplyObstacleConstraint(opti, xSegment(sampleIndex), ySegment(sampleIndex), thetaSegment(sampleIndex), bumpers, obstacle);
                }
                
            }
        }
    }

    void linspace(casadi::DM& arry, size_t startIndex, size_t endIndex, double startValue, double endValue) {
        size_t segmentCount = endIndex - startIndex;
        double delta = (endValue - startValue) / segmentCount;
        for (int index = 0; index < segmentCount; index++) {
            arry(startIndex + index) = startValue + index * delta;
        }
    }

    const casadi::DM CasADiTrajectoryOptimizationProblem::GenerateInitialGuessX(const Path& path) {
        size_t waypointCount = path.Length();
        casadi::DM X(3, path.ControlIntervalTotal() + 1);
        casadi::DM x = X(0, ALL);
        casadi::DM y = X(1, ALL);
        casadi::DM theta = X(2, ALL);
        size_t sampleIndex = path.GetWaypoint(0).controlIntervalCount;
        for (size_t waypointIndex = 1; waypointIndex < waypointCount; waypointIndex++) {
            const Waypoint& previousWaypoint = path.GetWaypoint(waypointIndex - 1);
            const Waypoint& waypoint = path.GetWaypoint(waypointIndex);
            size_t intervalCount = waypoint.controlIntervalCount;
            size_t guessPointCount = waypoint.initialGuessPoints.size();
            size_t previousWaypointSampleIndex = sampleIndex;
            size_t waypointSampleIndex = previousWaypointSampleIndex + intervalCount;
            if (guessPointCount == 0) {
                linspace(x, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.x, waypoint.x);
                linspace(y, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.y, waypoint.y);
                linspace(theta, previousWaypointSampleIndex, waypointSampleIndex, previousWaypoint.heading, waypoint.heading);
            } else {
                size_t guessSegmentIntervalCount = intervalCount / (guessPointCount + 1);
                linspace(x, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.x, waypoint.initialGuessPoints[0].x);
                linspace(y, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.y, waypoint.initialGuessPoints[0].y);
                linspace(theta, previousWaypointSampleIndex, previousWaypointSampleIndex + guessSegmentIntervalCount, previousWaypoint.heading, waypoint.initialGuessPoints[0].heading);
                size_t firstGuessPointSampleIndex = previousWaypointSampleIndex + guessSegmentIntervalCount;
                for (int guessPointIndex = 1; guessPointIndex < guessPointCount; guessPointIndex++) {
                    size_t previousGuessPointSampleIndex = firstGuessPointSampleIndex + (guessPointIndex - 1) * guessSegmentIntervalCount;
                    size_t guessPointSampleIndex = firstGuessPointSampleIndex + (guessPointIndex) * guessSegmentIntervalCount;
                    linspace(x, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].x, waypoint.initialGuessPoints[guessPointIndex].x);
                    linspace(y, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].y, waypoint.initialGuessPoints[guessPointIndex].y);
                    linspace(theta, previousGuessPointSampleIndex, guessPointSampleIndex, waypoint.initialGuessPoints[guessPointIndex - 1].heading, waypoint.initialGuessPoints[guessPointIndex].heading);
                }
                size_t finalGuessPointSampleIndex = previousWaypointSampleIndex + guessPointCount * guessSegmentIntervalCount;
                linspace(x, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].x, waypoint.x);
                linspace(y, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].y, waypoint.y);
                linspace(theta, finalGuessPointSampleIndex, waypointSampleIndex, waypoint.initialGuessPoints[guessPointCount - 1].heading, waypoint.heading);
            }
            sampleIndex += intervalCount;
        }
        x(sampleIndex) = path.GetWaypoint(waypointCount - 1).x;
        y(sampleIndex) = path.GetWaypoint(waypointCount - 1).y;
        theta(sampleIndex) = path.GetWaypoint(waypointCount - 1).heading;

        return X;
    }
}