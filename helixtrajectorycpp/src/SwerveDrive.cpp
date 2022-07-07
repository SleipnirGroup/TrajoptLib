#define _USE_MATH_DEFINES
#include <math.h>

#include "Obstacle.h"
#include "SwerveDrive.h"
#include "TrajectoryUtil.h"

namespace helixtrajectory {

    SwerveDrive::SwerveDrive(
            double wheelbaseX, double wheelbaseY, double length, double width,
            double mass, double moi,
            double wheelMaxAngularVelocity, double wheelMaxTorque, double wheelRadius) :
        HolonomicDrive(mass, moi,
            Obstacle(0, { {+length/2, +width/2}, {length/2, -width/2}, {-length/2, +width/2}, {-length/2, -width/2} })),
        modules({
            {+wheelbaseX, +wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque},
            {+wheelbaseX, -wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque},
            {-wheelbaseX, +wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque},
            {-wheelbaseX, -wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque} }) {
    }

    const casadi::MX SwerveDrive::SolveModulePositions(const casadi::MX& theta) const {
        size_t moduleCount = modules.size();
        casadi::MX positions(2, moduleCount);
        for (int i = 0; i < moduleCount; i++) {
            positions(0, i) = modules[i].GetModuleDiagonal() * cos(modules[i].GetModuleAngle() + theta);
            positions(1, i) = modules[i].GetModuleDiagonal() * sin(modules[i].GetModuleAngle() + theta);
        }
        return positions;
    }

    void SwerveDrive::ApplyKinematicsConstraints(casadi::Opti& opti,
            const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
            const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
            const casadi::MX& alpha, const size_t nTotal) const {
        for (int i = 0; i < nTotal; i++) {
            const casadi::MX modulePositions = SolveModulePositions(theta(i));
            double moduleCount = modules.size();
            casadi::MX forces = opti.variable(2, moduleCount);
            casadi::MX torques(1, moduleCount);

            for (int moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                casadi::MX moduleVx = vx(i) + modulePositions(0, moduleIndex) * omega(i);
                casadi::MX moduleVy = vy(i) + modulePositions(1, moduleIndex) * omega(i);
                double maxWheelVelocity = modules[i].wheelRadius * modules[i].wheelMaxAngularVelocity;
                opti.subject_to(moduleVx * moduleVx + moduleVy * moduleVy < maxWheelVelocity * maxWheelVelocity);
                torques(moduleIndex) = modulePositions(1, moduleIndex) * forces(0, moduleIndex) - modulePositions(0, moduleIndex) * forces(1, moduleIndex);
                double maxForce = modules[i].wheelMaxTorque / modules[i].wheelRadius;
                opti.subject_to(forces(0, moduleIndex) * forces(0, moduleIndex) + forces(1, moduleIndex) * forces(1, moduleIndex) < maxForce * maxForce);
            }
            opti.subject_to(ax(i) * mass == forces(0, 0) + forces(0, 1) + forces(0, 2) + forces(0, 3));
            opti.subject_to(ay(i) * mass == forces(1, 0) + forces(1, 1) + forces(1, 2) + forces(1, 3));
            opti.subject_to(alpha(i) * moi == torques(0) + torques(1) + torques(2) + torques(3));
        }
    }
}