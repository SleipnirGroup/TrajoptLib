#include "Drivetrain.h"

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    Drivetrain::Drivetrain(double mass, double momentOfInertia, const Obstacle& bumpers)
        : mass(mass), momentOfInertia(momentOfInertia), bumpers(bumpers) {
    }

    Drivetrain::~Drivetrain() {
    }

    const casadi::MX Drivetrain::SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, const ObstaclePoint& bumperCorner) {
        casadi::MX position(2, 1);
        double cornerDiagonal = hypot(bumperCorner.x, bumperCorner.y);
        double cornerAngle = atan2(bumperCorner.y, bumperCorner.x);
        if (cornerDiagonal == 0.0) {
            position(0) = x;
            position(1) = y;
        } else {
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

    void Drivetrain::ApplyObstacleConstraint(casadi::Opti& opti, const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, const Obstacle& obstacle) const {
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

    void Drivetrain::ApplyObstacleConstraints(casadi::Opti& opti, const std::vector<casadi::MX>& xSegments,
            const std::vector<casadi::MX>& ySegments, const std::vector<casadi::MX>& thetaSegments,
            const Path& path) const {
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
                    ApplyObstacleConstraint(opti, xSegment(sampleIndex), ySegment(sampleIndex), thetaSegment(sampleIndex), *obstacle);
                }
                for (const Obstacle& obstacle : waypoint.obstacles) {
                    ApplyObstacleConstraint(opti, xSegment(sampleIndex), ySegment(sampleIndex), thetaSegment(sampleIndex), obstacle);
                }
                
            }
        }
    }
}