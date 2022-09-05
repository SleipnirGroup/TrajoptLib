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
            moduleX(), moduleY(), moduleVX(), moduleVY(),
            moduleFX(), moduleFY(), moduleTau(),
            netFX(), netFY(), netTau() {


        moduleX.reserve(moduleCount);
        moduleY.reserve(moduleCount);
        moduleVX.reserve(moduleCount);
        moduleVY.reserve(moduleCount);
        moduleFX.reserve(moduleCount);
        moduleFY.reserve(moduleCount);
        moduleTau.reserve(moduleCount);
        netFX.reserve(controlIntervalTotal);
        netFY.reserve(controlIntervalTotal);
        netTau.reserve(controlIntervalTotal);

        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::vector<casadi::MX> indexModuleX;
            std::vector<casadi::MX> indexModuleY;
            std::vector<casadi::MX> indexModuleVX;
            std::vector<casadi::MX> indexModuleVY;
            std::vector<casadi::MX> indexModuleFX;
            std::vector<casadi::MX> indexModuleFY;
            std::vector<casadi::MX> indexModuleTau;
            indexModuleX.reserve(sampleTotal);
            indexModuleY.reserve(sampleTotal);
            indexModuleVX.reserve(sampleTotal);
            indexModuleVY.reserve(sampleTotal);
            indexModuleFX.reserve(controlIntervalTotal);
            indexModuleFY.reserve(controlIntervalTotal);
            indexModuleTau.reserve(controlIntervalTotal);
            for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
                ModulePosition modulePosition = SolveModulePosition(theta[sampleIndex],
                        CasADiSwerveTrajectoryOptimizationProblem::swerveDrivetrain.modules[moduleIndex]);
                indexModuleX.push_back(modulePosition.x);
                indexModuleY.push_back(modulePosition.y);
                indexModuleVX.push_back(vx[sampleIndex] - indexModuleY[sampleIndex] * omega[sampleIndex]);
                indexModuleVY.push_back(vy[sampleIndex] + indexModuleX[sampleIndex] * omega[sampleIndex]);
            }
            for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
                indexModuleFX.push_back(opti.variable());
                indexModuleFY.push_back(opti.variable());
                indexModuleTau.push_back(indexModuleX[intervalIndex + 1] * indexModuleFY[intervalIndex]
                        - indexModuleY[intervalIndex + 1] * indexModuleFX[intervalIndex]);
            }
            moduleX.push_back(indexModuleX);
            moduleY.push_back(indexModuleY);
            moduleVX.push_back(indexModuleVX);
            moduleVY.push_back(indexModuleVY);
            moduleFX.push_back(indexModuleFX);
            moduleFY.push_back(indexModuleFY);
            moduleTau.push_back(indexModuleTau);
        }

#ifdef DEBUG_OUTPUT
        std::cout << "Set up module position and velocity variables" << std::endl;
#endif

        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            casadi::MX intervalNetFX = 0;
            casadi::MX intervalNetFY = 0;
            casadi::MX intervalNetTau = 0;
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                intervalNetFX += moduleFX[moduleIndex][intervalIndex];
                intervalNetFY += moduleFY[moduleIndex][intervalIndex];
                intervalNetTau += moduleTau[moduleIndex][intervalIndex];
            }
            netFX.push_back(intervalNetFX);
            netFY.push_back(intervalNetFY);
            netTau.push_back(intervalNetTau);
        }

#ifdef DEBUG_OUTPUT
        std::cout << "Set up net force and net torque expressions" << std::endl;
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

    const CasADiSwerveTrajectoryOptimizationProblem::ModulePosition CasADiSwerveTrajectoryOptimizationProblem::SolveModulePosition(const casadi::MX& theta, const SwerveModule& module) {
        ModulePosition position{};
        if (module.x == 0.0 && module.y == 0.0) {
            position.x = 0;
            position.y = 0;
        } else {
            double moduleDiagonal = hypot(module.x, module.y);
            double moduleAngle = atan2(module.y, module.x);
            position.x = moduleDiagonal * cos(moduleAngle + theta);
            position.y = moduleDiagonal * sin(moduleAngle + theta);
        }
        return position;
    }

    void CasADiSwerveTrajectoryOptimizationProblem::ApplyDynamicsConstraints(casadi::Opti& opti,
            const std::vector<casadi::MX>& ax, const std::vector<casadi::MX>& ay, const std::vector<casadi::MX>& alpha,
            const std::vector<casadi::MX>& netFX, const std::vector<casadi::MX>& netFY, const std::vector<casadi::MX>& netTau,
            const SwerveDrivetrain& swerveDrivetrain) {
        size_t controlIntervalTotal = ax.size();
        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            opti.subject_to(netFX[intervalIndex]  == swerveDrivetrain.mass            * ax[intervalIndex]);
            opti.subject_to(netFY[intervalIndex]  == swerveDrivetrain.mass            * ay[intervalIndex]);
            opti.subject_to(netTau[intervalIndex] == swerveDrivetrain.momentOfInertia * alpha[intervalIndex]);
        }
    }

    void CasADiSwerveTrajectoryOptimizationProblem::ApplyPowerConstraints(casadi::Opti& opti,
            const std::vector<std::vector<casadi::MX>>& moduleVX, const std::vector<std::vector<casadi::MX>>& moduleVY,
            const std::vector<std::vector<casadi::MX>>& moduleFX, const std::vector<std::vector<casadi::MX>>& moduleFY,
            const SwerveDrivetrain& swerveDrivetrain) {
        size_t moduleCount = swerveDrivetrain.modules.size();
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            size_t controlIntervalTotal = moduleVX[moduleIndex].size() - 1;
            const SwerveModule& _module = swerveDrivetrain.modules[moduleIndex];
            double maxWheelVelocity = _module.wheelRadius * _module.wheelMaxAngularVelocity;
            for (size_t sampleIndex = 0; sampleIndex < controlIntervalTotal + 1; sampleIndex++) {
                casadi::MX constraint = moduleVX[moduleIndex][sampleIndex] * moduleVX[moduleIndex][sampleIndex]
                              + moduleVY[moduleIndex][sampleIndex] * moduleVY[moduleIndex][sampleIndex]
                             <= maxWheelVelocity * maxWheelVelocity;
                std::cout << "\n\nMax velocity constraint: " << constraint << std::endl;
                opti.subject_to(moduleVX[moduleIndex][sampleIndex] * moduleVX[moduleIndex][sampleIndex]
                              + moduleVY[moduleIndex][sampleIndex] * moduleVY[moduleIndex][sampleIndex]
                             <= maxWheelVelocity * maxWheelVelocity);
                
                // std::cout << "Applied module " << moduleIndex << " sample " << sampleIndex << " velocity power constraint of " << maxWheelVelocity << std::endl;
            }

            double maxForce = _module.wheelMaxTorque / _module.wheelRadius;
            for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
                opti.subject_to(moduleFX[moduleIndex][intervalIndex] * moduleFX[moduleIndex][intervalIndex]
                              + moduleFY[moduleIndex][intervalIndex] * moduleFY[moduleIndex][intervalIndex]
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
        // printCSV(horzcat(
        //                  horzcat(
        //                      vertcat(casadi::DM(1, 1), solution.value(dt).T()),
        //                      solution.value(X).T(),
        //                      solution.value(V).T(),
        //                      vertcat(casadi::DM(1, 3), solution.value(U).T())),
        //                  horzcat(
        //                     solution.value(moduleX).T(),
        //                     solution.value(moduleY).T(),
        //                     solution.value(moduleVX).T(),
        //                     solution.value(moduleVY).T()),
        //                  horzcat(
        //                     vertcat(casadi::DM(1, moduleCount), solution.value(moduleFX).T()),
        //                     vertcat(casadi::DM(1, moduleCount), solution.value(moduleFY).T()),
        //                     vertcat(casadi::DM(1, moduleCount), solution.value(moduleTau).T())),
        //                  horzcat(
        //                     vertcat(casadi::DM(1, 1), solution.value(netFX).T()),
        //                     vertcat(casadi::DM(1, 1), solution.value(netFY).T()),
        //                     vertcat(casadi::DM(1, 1), solution.value(netTau).T()))));
#endif
    }
}