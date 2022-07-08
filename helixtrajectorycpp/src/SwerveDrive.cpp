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
            Obstacle(0, { {+length/2, +width/2}, {-length/2, +width/2}, {-length/2, -width/2}, {+length/2, -width/2} })), // full four corners
            // Obstacle(0, { {+length/2, +width/2}, {length/2, -width/2} })), // only front two
            // Obstacle(hypot(length / 2, width / 2), { {0, 0} })), // approximate with circle
        modules({
            {+wheelbaseX, +wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque},
            {+wheelbaseX, -wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque},
            {-wheelbaseX, +wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque},
            {-wheelbaseX, -wheelbaseY, wheelRadius, wheelMaxAngularVelocity, wheelMaxTorque} }) {
    }

    const casadi::MX SwerveDrive::SolveModulePosition(const casadi::MX& theta, const SwerveModule& module) const {
        casadi::MX position(2, 1);
        position(0) = module.GetModuleDiagonal() * cos(module.GetModuleAngle() + theta);
        position(1) = module.GetModuleDiagonal() * sin(module.GetModuleAngle() + theta);
        return position;
    }

    void SwerveDrive::ApplyKinematicsConstraints(casadi::Opti& opti,
            const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
            const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
            const casadi::MX& alpha, const size_t nTotal) const {
        for (int i = 0; i < nTotal; i++) {
            std::cout << "Constraining module kinematics" << std::endl;
            double moduleCount = modules.size();
            casadi::MX forces = opti.variable(2, moduleCount);
            casadi::MX torques(1, moduleCount);
            casadi::MX netForceX = 0;
            casadi::MX netForceY = 0;
            casadi::MX netTorque = 0;
            std::cout << "Set up module variables" << std::endl;

            for (int moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                std::cout << "about to solve module position" << std::endl;
                const casadi::MX modulePosition = SolveModulePosition(theta(i), modules[moduleIndex]);
                std::cout << "Solved module positions" << std::endl;
                casadi::MX moduleVx = vx(i) + modulePosition(0) * omega(i);
                casadi::MX moduleVy = vy(i) + modulePosition(1) * omega(i);
                double maxWheelVelocity = modules[moduleIndex].wheelRadius * modules[moduleIndex].wheelMaxAngularVelocity;
                opti.subject_to(moduleVx * moduleVx + moduleVy * moduleVy <= maxWheelVelocity * maxWheelVelocity);
                std::cout << "Constrained module velocity" << std::endl;
                torques(moduleIndex) = modulePosition(1) * forces(0, moduleIndex) - modulePosition(0) * forces(1, moduleIndex);
                double maxForce = modules[moduleIndex].wheelMaxTorque / modules[moduleIndex].wheelRadius;
                opti.subject_to(forces(0, moduleIndex) * forces(0, moduleIndex) + forces(1, moduleIndex) * forces(1, moduleIndex) <= maxForce * maxForce);
                std::cout << "Constrained module forces" << std::endl;
                netForceX += forces(0, moduleIndex);
                netForceY += forces(1, moduleIndex);
                netTorque += torques(all, moduleIndex);
            }
            opti.subject_to(ax(i) * mass == netForceX);
            opti.subject_to(ay(i) * mass == netForceY);
            opti.subject_to(alpha(i) * moi == netTorque);
            std::cout << "Constrained module to kinematics" << std::endl;
        }
    }
}