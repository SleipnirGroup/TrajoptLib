#define _USE_MATH_DEFINES
#include <math.h>

#include "SwerveDrive.h"

namespace helixtrajectory {
    SwerveDrive::SwerveDrive(const double wheelbaseX, const double wheelbaseY,
            const double length, const double width, const double mass,
            const double moi, const double omegaMax, const double tauMax,
            const double wheelRadius) :
        wheelbaseX(wheelbaseX), wheelbaseY(wheelbaseY), length(length), width(width),
        mass(mass), moi(moi), omegaMax(omegaMax), tauMax(tauMax), wheelRadius(wheelRadius),
        maxWheelVelocitySquared((omegaMax * wheelRadius) * (omegaMax * wheelRadius)),
        maxForceSquared((tauMax / wheelRadius) * (tauMax / wheelRadius)),
        moduleAngle(atan2(wheelbaseY, wheelbaseX)),
        diagonal(hypot(wheelbaseX, wheelbaseY)) {}

    const casadi::MX SwerveDrive::SolveModulePositions(const casadi::MX& theta) const {
        double moduleAngles[] = { moduleAngle, -moduleAngle, M_PI - moduleAngle, -(M_PI - moduleAngle)};
        casadi::MX positions(2, 4);
        for (int i = 0; i < 4; i++) {
            positions(0, i) = diagonal * cos(moduleAngles[i] + theta);
            positions(1, i) = diagonal * sin(moduleAngles[i] + theta);
        }
        return positions;
    }

    void SwerveDrive::ApplyKinematicsConstraints(casadi::Opti& solver,
            const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
            const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
            const casadi::MX& alpha, const size_t nTotal) const {
        for (int i = 0; i < nTotal; i++) {
            const casadi::MX modulePositions = SolveModulePositions(theta(i));
            casadi::MX forces = solver.variable(2, 4);
            casadi::MX torques(1, 4);

            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                casadi::MX moduleVx = vx(i) + modulePositions(0, moduleIndex) * omega(i);
                casadi::MX moduleVy = vy(i) + modulePositions(1, moduleIndex) * omega(i);
                solver.subject_to(moduleVx * moduleVx + moduleVy * moduleVy < maxWheelVelocitySquared);
                torques(moduleIndex) = modulePositions(1, moduleIndex) * forces(0, moduleIndex) - modulePositions(0, moduleIndex) * forces(1, moduleIndex);
                solver.subject_to(forces(0, moduleIndex) * forces(0, moduleIndex) + forces(1, moduleIndex) * forces(1, moduleIndex) < maxForceSquared);
            }
            
            solver.subject_to(ax(i) * mass == forces(0, 0) + forces(0, 1) + forces(0, 2) + forces(0, 3));
            solver.subject_to(ay(i) * mass == forces(1, 0) + forces(1, 1) + forces(1, 2) + forces(1, 3));
            solver.subject_to(alpha(i) * moi == torques(0) + torques(1) + torques(2) + torques(3));
        }
    }
}