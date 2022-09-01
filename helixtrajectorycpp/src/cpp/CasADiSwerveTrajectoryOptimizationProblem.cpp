#include "DebugOptions.h"

#include "CasADiSwerveTrajectoryOptimizationProblem.h"

#include <iostream>

#include "HolonomicPath.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    CasADiSwerveTrajectoryOptimizationProblem::CasADiSwerveTrajectoryOptimizationProblem(
            const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath)
            : CasADiHolonomicTrajectoryOptimizationProblem(swerveDrivetrain, holonomicPath),
            swerveDrivetrain(swerveDrivetrain), moduleCount(swerveDrivetrain.modules.size()),
            moduleX(moduleCount, controlIntervalTotal + 1), moduleY(moduleCount, controlIntervalTotal + 1),
            moduleVX(moduleCount, controlIntervalTotal + 1), moduleVY(moduleCount, controlIntervalTotal + 1),
            moduleFX(opti.variable(moduleCount, controlIntervalTotal)), moduleFY(opti.variable(moduleCount, controlIntervalTotal)),
            moduleTau(moduleCount, controlIntervalTotal), netFX(1, controlIntervalTotal), netFY(1, controlIntervalTotal),
            netTau(1, controlIntervalTotal) {

        for (size_t sampleIndex = 0; sampleIndex < controlIntervalTotal + 1; sampleIndex++) {
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                const casadi::MX modulePosition = SolveModulePosition(theta(sampleIndex), swerveDrivetrain.modules[moduleIndex]);
                moduleX(moduleIndex, sampleIndex) = modulePosition(0);
                moduleY(moduleIndex, sampleIndex) = modulePosition(1);
                moduleVX(moduleIndex, sampleIndex) = vx(sampleIndex) + moduleX(moduleIndex, sampleIndex) * omega(sampleIndex);
                moduleVY(moduleIndex, sampleIndex) = vy(sampleIndex) + moduleY(moduleIndex, sampleIndex) * omega(sampleIndex);
            }
        }
#ifdef DEBUG_OUTPUT
        std::cout << "Set up module position and velocity variables" << std::endl;
#endif

        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            casadi::MX intervalNetFX = 0;
            casadi::MX intervalNetFY = 0;
            casadi::MX intervalNetTau = 0;
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                // std::cout << "set up one torque variable" << std::endl;
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

#ifdef DEBUG_OUTPUT
        std::cout << "Set up swerve force variables" << std::endl;
#endif

        ApplyDynamicsConstraints(opti, ax, ay, alpha, netFX, netFY, netTau,
                CasADiSwerveTrajectoryOptimizationProblem::swerveDrivetrain);

#ifdef DEBUG_OUTPUT
        std::cout << "Applied swerve dynamics constraints" << std::endl;
#endif

        ApplyPowerConstraints(opti, moduleVX, moduleVY, moduleFX, moduleFY,
                CasADiSwerveTrajectoryOptimizationProblem::swerveDrivetrain);

#ifdef DEBUG_OUTPUT
        std::cout << "Applied swerve power constraints" << std::endl;
#endif
    }

    const casadi::MX CasADiSwerveTrajectoryOptimizationProblem::SolveModulePosition(const casadi::MX& theta, const SwerveModule& module) {
        casadi::MX position(2, 1);
        if (module.x == 0.0 && module.y == 0.0) {
            position(0) = 0;
            position(1) = 0;
        } else {
            double moduleDiagonal = hypot(module.x, module.y);
            double moduleAngle = atan2(module.y, module.x);
            position(0) = moduleDiagonal * cos(moduleAngle + theta);
            position(1) = moduleDiagonal * sin(moduleAngle + theta);
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
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            const SwerveModule& _module = swerveDrivetrain.modules[moduleIndex];
            double maxWheelVelocity = _module.wheelRadius * _module.wheelMaxAngularVelocity;
            for (size_t sampleIndex = 0; sampleIndex < controlIntervalTotal + 1; sampleIndex++) {
                opti.subject_to(moduleVX(moduleIndex, sampleIndex) * moduleVX(moduleIndex, sampleIndex)
                              + moduleVY(moduleIndex, sampleIndex) * moduleVY(moduleIndex, sampleIndex)
                             <= maxWheelVelocity * maxWheelVelocity);
                
                // std::cout << "Applied module " << moduleIndex << " sample " << sampleIndex << " velocity power constraint of " << maxWheelVelocity << std::endl;
            }

            double maxForce = _module.wheelMaxTorque / _module.wheelRadius;
            for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
                opti.subject_to(moduleFX(moduleIndex, intervalIndex) * moduleFX(moduleIndex, intervalIndex)
                              + moduleFY(moduleIndex, intervalIndex) * moduleFY(moduleIndex, intervalIndex)
                              <= maxForce * maxForce);
                // std::cout << "Applied module " << moduleIndex << " interval " << intervalIndex << " force power constraint of " << maxForce << std::endl;
            }
        }
    }

#ifdef DEBUG_OUTPUT
    void printCSV(const casadi::DM& matrix) {
        size_t rows = matrix.rows();
        size_t columns = matrix.columns();
        for (size_t row = 0; row < rows; row++) {
            if (columns > 0) {
                std::cout << "\n" << matrix(row, 0);
            }
            for (size_t column = 1; column < columns; column++) {
                std::cout << ", " << matrix(row, column);
            }
        }
    }
#endif

    void CasADiSwerveTrajectoryOptimizationProblem::PrintSolution(const casadi::OptiSol& solution) const {
        std::cout << "dt, x, y, theta, vx, vy, omega, ax, ay, alpha";
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "x";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "y";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "theta";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "vx";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "vy";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "omega";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "fx";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "fy";
        }
#ifdef DEBUG_OUTPUT
        printCSV(horzcat(
                         horzcat(
                             vertcat(casadi::DM(1, 1), solution.value(dt).T()),
                             solution.value(X).T(),
                             solution.value(V).T(),
                             vertcat(casadi::DM(1, 3), solution.value(U).T())),
                         horzcat(
                            solution.value(moduleX).T(),
                            solution.value(moduleY).T(),
                            solution.value(moduleVX).T(),
                            solution.value(moduleVY).T()),
                         horzcat(
                            vertcat(casadi::DM(1, moduleCount), solution.value(moduleFX).T()),
                            vertcat(casadi::DM(1, moduleCount), solution.value(moduleFY).T()),
                            vertcat(casadi::DM(1, moduleCount), solution.value(moduleTau).T())),
                         horzcat(
                            vertcat(casadi::DM(1, 1), solution.value(netFX).T()),
                            vertcat(casadi::DM(1, 1), solution.value(netFY).T()),
                            vertcat(casadi::DM(1, 1), solution.value(netTau).T()))));
#endif
    }
}