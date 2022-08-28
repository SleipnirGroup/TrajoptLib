#include "CasADiSwerveTrajectoryOptimizationProblem.h"

#include "HolonomicPath.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    CasADiSwerveTrajectoryOptimizationProblem::CasADiSwerveTrajectoryOptimizationProblem(
            const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath)
            : CasADiHolonomicTrajectoryOptimizationProblem(swerveDrivetrain, holonomicPath),
            swerveDrivetrain(swerveDrivetrain), moduleCount(swerveDrivetrain.modules.size()),
            moduleX(moduleCount, controlIntervalTotal + 1), moduleY(moduleCount, controlIntervalTotal + 1),
            moduleVX(moduleCount, controlIntervalTotal + 1), moduleVY(moduleCount, controlIntervalTotal + 1),
            moduleFX(opti.variable(moduleCount, controlIntervalTotal)), moduleFY(opti.variable(moduleCount, controlIntervalTotal)) {

        for (size_t sampleIndex = 0; sampleIndex < controlIntervalTotal + 1; sampleIndex++) {
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                const casadi::MX modulePosition = SolveModulePosition(theta(sampleIndex), swerveDrivetrain.modules[moduleIndex]);
                moduleX(moduleIndex, sampleIndex) = modulePosition(0);
                moduleY(moduleIndex, sampleIndex) = modulePosition(1);
                moduleVX(moduleIndex, sampleIndex) = vx(sampleIndex) + moduleX(moduleIndex, sampleIndex) * omega(sampleIndex);
                moduleVY(moduleIndex, sampleIndex) = vy(sampleIndex) + moduleY(moduleIndex, sampleIndex) * omega(sampleIndex);
            }
        }

        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            casadi::MX intervalNetFX = 0;
            casadi::MX intervalNetFY = 0;
            casadi::MX intervalNetTau = 0;
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                moduleTau(moduleIndex, intervalIndex)
                        = moduleX(moduleIndex, intervalIndex + 1) * moduleFY(moduleIndex, intervalIndex)
                        - moduleY(moduleIndex, intervalIndex + 1) * moduleFX(moduleIndex, intervalIndex);
                intervalNetFX += moduleFX(moduleIndex, intervalIndex);
                intervalNetFY += moduleFY(moduleIndex, intervalIndex);
                intervalNetTau += moduleTau(moduleIndex, intervalIndex);
            }
            netFX(intervalIndex) = intervalNetFX;
            netFY(intervalIndex) = intervalNetFY;
            netTau(intervalIndex) = intervalNetTau;
        }
    }

    const casadi::MX CasADiSwerveTrajectoryOptimizationProblem::SolveModulePosition(const casadi::MX& theta, const SwerveModule& module) {
        casadi::MX position(2, 1);
        if (module.x == 0.0 && module.y == 0.0) {
            position(0) = 0;
            position(1) = 0;
        } else {
            position(0) = module.GetModuleDiagonal() * cos(module.GetModuleAngle() + theta);
            position(1) = module.GetModuleDiagonal() * sin(module.GetModuleAngle() + theta);
        }
        return position;
    }

    void CasADiSwerveTrajectoryOptimizationProblem::ApplyDynamicsConstraints(casadi::Opti& opti,
                const casadi::MX& ax, const casadi::MX& ay, const casadi::MX& alpha,
                const casadi::MX& netFX, const casadi::MX& netFY, const casadi::MX& netTau,
                const SwerveDrivetrain& swerveDrivetrain) {
        opti.subject_to(netFX == swerveDrivetrain.mass * ax);
        opti.subject_to(netFY == swerveDrivetrain.mass * ay);
        opti.subject_to(netTau == swerveDrivetrain.momentOfInertia * alpha);
    }

    void CasADiSwerveTrajectoryOptimizationProblem::ApplyPowerConstraints(casadi::Opti& opti,
            const casadi::MX& moduleVX, const casadi::MX& moduleVY,
            const casadi::MX& moduleFX, const casadi::MX& moduleFY,
            const SwerveDrivetrain& swerveDrivetrain) {

        size_t controlIntervalTotal = moduleVX.columns() - 1;
        size_t moduleCount = swerveDrivetrain.modules.size();
        for (size_t sampleIndex = 0; sampleIndex < controlIntervalTotal + 1; sampleIndex++) {
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                const SwerveModule& _module = swerveDrivetrain.modules[moduleIndex];
                double maxWheelVelocity = _module.wheelRadius * _module.wheelMaxAngularVelocity;
                opti.subject_to(moduleVX(moduleIndex, sampleIndex) * moduleVX(moduleIndex, sampleIndex)
                              + moduleVY(moduleIndex, sampleIndex) * moduleVY(moduleIndex, sampleIndex)
                             <= maxWheelVelocity * maxWheelVelocity);
            }
        }
        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                const SwerveModule& _module = swerveDrivetrain.modules[moduleIndex];
                double maxForce = _module.wheelMaxTorque / _module.wheelRadius;
                opti.subject_to(moduleFX(moduleIndex, intervalIndex) * moduleFX(moduleIndex, intervalIndex)
                              + moduleFY(moduleIndex, intervalIndex) * moduleFY(moduleIndex, intervalIndex) <= maxForce * maxForce);
            }
        }
    }
}