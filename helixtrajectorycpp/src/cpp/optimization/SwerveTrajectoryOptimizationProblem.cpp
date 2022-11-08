#include "DebugOptions.h"

#include "SwerveTrajectoryOptimizationProblem.h"

#include <iostream>

#include "CasADiOpti.h"
#include "HolonomicPath.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    template<typename Opti>
    SwerveTrajectoryOptimizationProblem<Opti>::SwerveTrajectoryOptimizationProblem(
            const SwerveDrivetrain& swerveDrivetrain, const HolonomicPath& holonomicPath)
            : HolonomicTrajectoryOptimizationProblem<Opti>(swerveDrivetrain, holonomicPath),
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
        netFX.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
        netFY.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
        netTau.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);

        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::vector<Expression> indexModuleX;
            std::vector<Expression> indexModuleY;
            std::vector<Expression> indexModuleVX;
            std::vector<Expression> indexModuleVY;
            std::vector<Expression> indexModuleFX;
            std::vector<Expression> indexModuleFY;
            std::vector<Expression> indexModuleTau;
            indexModuleX.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
            indexModuleY.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
            indexModuleVX.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
            indexModuleVY.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
            indexModuleFX.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
            indexModuleFY.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
            indexModuleTau.reserve(TrajectoryOptimizationProblem<Opti>::controlIntervalTotal);
            for (size_t sampleIndex = 0; sampleIndex < TrajectoryOptimizationProblem<Opti>::sampleTotal; sampleIndex++) {
                ModulePosition modulePosition = SolveModulePosition(TrajectoryOptimizationProblem<Opti>::theta[sampleIndex],
                        SwerveTrajectoryOptimizationProblem::swerveDrivetrain.modules[moduleIndex]);
                indexModuleX.push_back(modulePosition.x);
                indexModuleY.push_back(modulePosition.y);
                indexModuleVX.push_back(HolonomicTrajectoryOptimizationProblem<Opti>::vx[sampleIndex] - indexModuleY[sampleIndex] * HolonomicTrajectoryOptimizationProblem<Opti>::omega[sampleIndex]);
                indexModuleVY.push_back(HolonomicTrajectoryOptimizationProblem<Opti>::vy[sampleIndex] + indexModuleX[sampleIndex] * HolonomicTrajectoryOptimizationProblem<Opti>::omega[sampleIndex]);
            }
            for (size_t intervalIndex = 0; intervalIndex < TrajectoryOptimizationProblem<Opti>::controlIntervalTotal; intervalIndex++) {
                indexModuleFX.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
                indexModuleFY.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
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

        for (size_t intervalIndex = 0; intervalIndex < TrajectoryOptimizationProblem<Opti>::controlIntervalTotal; intervalIndex++) {
            Expression intervalNetFX = 0;
            Expression intervalNetFY = 0;
            Expression intervalNetTau = 0;
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

        ApplyDynamicsConstraints(TrajectoryOptimizationProblem<Opti>::opti,
                HolonomicTrajectoryOptimizationProblem<Opti>::ax,
                HolonomicTrajectoryOptimizationProblem<Opti>::ay,
                HolonomicTrajectoryOptimizationProblem<Opti>::alpha, netFX, netFY, netTau,
                SwerveTrajectoryOptimizationProblem::swerveDrivetrain);

#ifdef DEBUG_OUTPUT
        std::cout << "Applied swerve dynamics constraints" << std::endl;
#endif

        ApplyPowerConstraints(TrajectoryOptimizationProblem<Opti>::opti, moduleVX, moduleVY, moduleFX, moduleFY,
                SwerveTrajectoryOptimizationProblem::swerveDrivetrain);

#ifdef DEBUG_OUTPUT
        std::cout << "Applied swerve power constraints" << std::endl;
#endif
    }

    template<typename Opti>
    const typename SwerveTrajectoryOptimizationProblem<Opti>::ModulePosition SwerveTrajectoryOptimizationProblem<Opti>::SolveModulePosition(const Expression& theta, const SwerveModule& module) {
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

    template<typename Opti>
    void SwerveTrajectoryOptimizationProblem<Opti>::ApplyDynamicsConstraints(Opti& opti,
            const std::vector<Expression>& ax, const std::vector<Expression>& ay, const std::vector<Expression>& alpha,
            const std::vector<Expression>& netFX, const std::vector<Expression>& netFY, const std::vector<Expression>& netTau,
            const SwerveDrivetrain& swerveDrivetrain) {
        size_t controlIntervalTotal = ax.size();
        for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
            opti.SubjectTo(netFX[intervalIndex]  == swerveDrivetrain.mass            * ax[intervalIndex]);
            opti.SubjectTo(netFY[intervalIndex]  == swerveDrivetrain.mass            * ay[intervalIndex]);
            opti.SubjectTo(netTau[intervalIndex] == swerveDrivetrain.momentOfInertia * alpha[intervalIndex]);
        }
    }

    template<typename Opti>
    void SwerveTrajectoryOptimizationProblem<Opti>::ApplyPowerConstraints(Opti& opti,
            const std::vector<std::vector<Expression>>& moduleVX, const std::vector<std::vector<Expression>>& moduleVY,
            const std::vector<std::vector<Expression>>& moduleFX, const std::vector<std::vector<Expression>>& moduleFY,
            const SwerveDrivetrain& swerveDrivetrain) {
        size_t moduleCount = swerveDrivetrain.modules.size();
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            size_t controlIntervalTotal = moduleVX[moduleIndex].size() - 1;
            const SwerveModule& _module = swerveDrivetrain.modules[moduleIndex];
            double maxWheelVelocity = _module.wheelRadius * _module.wheelMaxAngularVelocity;
            for (size_t sampleIndex = 0; sampleIndex < controlIntervalTotal + 1; sampleIndex++) {
                Expression constraint = moduleVX[moduleIndex][sampleIndex] * moduleVX[moduleIndex][sampleIndex]
                              + moduleVY[moduleIndex][sampleIndex] * moduleVY[moduleIndex][sampleIndex]
                             <= maxWheelVelocity * maxWheelVelocity;
                // std::cout << "\n\nMax velocity constraint: " << constraint << std::endl;
                opti.SubjectTo(moduleVX[moduleIndex][sampleIndex] * moduleVX[moduleIndex][sampleIndex]
                              + moduleVY[moduleIndex][sampleIndex] * moduleVY[moduleIndex][sampleIndex]
                             <= maxWheelVelocity * maxWheelVelocity);
                
                // std::cout << "Applied module " << moduleIndex << " sample " << sampleIndex << " velocity power constraint of " << maxWheelVelocity << std::endl;
            }

            double maxForce = _module.wheelMaxTorque / _module.wheelRadius;
            for (size_t intervalIndex = 0; intervalIndex < controlIntervalTotal; intervalIndex++) {
                opti.SubjectTo(moduleFX[moduleIndex][intervalIndex] * moduleFX[moduleIndex][intervalIndex]
                              + moduleFY[moduleIndex][intervalIndex] * moduleFY[moduleIndex][intervalIndex]
                              <= maxForce * maxForce);
                // std::cout << "Applied module " << moduleIndex << " interval " << intervalIndex << " force power constraint of " << maxForce << std::endl;
            }
        }
    }

#ifdef DEBUG_OUTPUT
    template<typename Opti>
    void SwerveTrajectoryOptimizationProblem<Opti>::PrintSolution() const {
        std::cout << "Printing " << TrajectoryOptimizationProblem<Opti>::sampleTotal << " samples:\n\n";
        std::cout << "sample, dt, x, y, theta, vx, vy, omega, ax, ay, alpha";
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "x";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "y";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "vx";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "vy";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "fx";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "fy";
        }
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            std::cout << ", module" << moduleIndex << "tau";
        }
        std::cout << "\n";

        for (size_t sampleIndex = 0; sampleIndex < TrajectoryOptimizationProblem<Opti>::sampleTotal; sampleIndex++) {
            std::cout << sampleIndex;
            if (sampleIndex == 0) {
                std::cout << ",";
            } else {
                std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(TrajectoryOptimizationProblem<Opti>::dt[sampleIndex - 1]);
            }
            std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(          TrajectoryOptimizationProblem<Opti>::x[sampleIndex]) << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(          TrajectoryOptimizationProblem<Opti>::y[sampleIndex]) << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(         TrajectoryOptimizationProblem<Opti>::theta[sampleIndex])
                    << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(HolonomicTrajectoryOptimizationProblem<Opti>::vx[sampleIndex]) << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(HolonomicTrajectoryOptimizationProblem<Opti>::vy[sampleIndex]) << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(HolonomicTrajectoryOptimizationProblem<Opti>::omega[sampleIndex]);
            if (sampleIndex == 0) {
                std::cout << ",,,";
            } else {
                std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(HolonomicTrajectoryOptimizationProblem<Opti>::ax[sampleIndex - 1])
                        << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(HolonomicTrajectoryOptimizationProblem<Opti>::ay[sampleIndex - 1])
                        << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(HolonomicTrajectoryOptimizationProblem<Opti>::alpha[sampleIndex - 1]);
            }

            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleX[moduleIndex][sampleIndex]);
            }
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleY[moduleIndex][sampleIndex]);
            }
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleVX[moduleIndex][sampleIndex]);
            }
            for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleVY[moduleIndex][sampleIndex]);
            }
            
            if (sampleIndex == 0) {
                for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                    std::cout << ",,,";
                }
            } else {
                for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                    std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleFX[moduleIndex][sampleIndex - 1]);
                }
                for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                    std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleFY[moduleIndex][sampleIndex - 1]);
                }
                for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
                    std::cout << ", " << TrajectoryOptimizationProblem<Opti>::opti.SolutionValue(moduleTau[moduleIndex][sampleIndex - 1]);
                }
            }
            std::cout << "\n";
        }
    }
#endif

    template class SwerveTrajectoryOptimizationProblem<CasADiOpti>;
}