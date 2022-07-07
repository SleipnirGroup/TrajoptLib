#include "Drive.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {

    Drive::Drive(double mass, double moi, const Obstacle& bumpers)
        : mass(mass), moi(moi), bumpers(bumpers) {
    }

    const casadi::MX Drive::SolveBumperCornerPosition(const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, const ObstaclePoint& bumperCorner) const {
        casadi::MX position(2);
        double cornerDiagonal = hypot(bumperCorner.x, bumperCorner.y);
        double cornerAngle = atan2(bumperCorner.y, bumperCorner.x);
        position(0) = x + cornerDiagonal * cos(cornerAngle + theta);
        position(1) = y + cornerDiagonal * sin(cornerAngle + theta);
        return position;
    }

    void Drive::ApplyObstacleConstraints(casadi::Opti& opti, const casadi::MX& x, const casadi::MX& y,
            const casadi::MX& theta, size_t nTotal, const std::vector<Obstacle>& obstacles) const {
        for (size_t i = 0; i < nTotal; i++) {
            for (const Obstacle& obstacle : obstacles) {
                size_t bumperCornerCount = bumpers.points.size();
                for (size_t bumperCornerIndex = 0; bumperCornerIndex < bumperCornerCount - 1; bumperCornerIndex++) {
                    casadi::MX startBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerIndex]);
                    casadi::MX endBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerIndex + 1]);
                    for (const ObstaclePoint& obstaclePoint : obstacle.points) {
                        casadi::MX dist = linePointDist(startBumperCornerPosition(0), startBumperCornerPosition(1),
                                endBumperCornerPosition(0), endBumperCornerPosition(1), obstaclePoint.x, obstaclePoint.y);
                        opti.subject_to(dist >= bumpers.safetyDistance + obstacle.safetyDistance);
                    }
                }
                if (bumperCornerCount >= 3) { // must have at least three points to make a closed shape
                    casadi::MX startBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[bumperCornerCount - 1]);
                    casadi::MX endBumperCornerPosition = SolveBumperCornerPosition(x, y, theta, bumpers.points[0]);
                    for (const ObstaclePoint& obstaclePoint : obstacle.points) {
                        casadi::MX dist = linePointDist(startBumperCornerPosition(0), startBumperCornerPosition(1),
                                endBumperCornerPosition(0), endBumperCornerPosition(1), obstaclePoint.x, obstaclePoint.y);
                        opti.subject_to(dist >= bumpers.safetyDistance + obstacle.safetyDistance);
                    }
                }

                size_t obstacleCornerCount = obstacle.points.size();
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
    }
}