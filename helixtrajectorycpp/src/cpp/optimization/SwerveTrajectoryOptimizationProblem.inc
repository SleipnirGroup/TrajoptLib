#include "DebugOptions.h"

// #include "optimization/SwerveTrajectoryOptimizationProblem.h"

#include <iostream>

#include "TrajectoryGenerationException.h"
#include "path/HolonomicPath.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "solution/SwerveSolution.h"

namespace helixtrajectory {

template<typename Opti>
SwerveSolution SwerveTrajectoryOptimizationProblem<Opti>::Generate() {
    try {
        TrajectoryOptimizationProblem<Opti>::opti.Solve();
#ifdef DEBUG_OUTPUT
        std::cout << "Solution stored" << std::endl;
#endif
        return ConstructSwerveSolution(
                TrajectoryOptimizationProblem<Opti>::opti,
                TrajectoryOptimizationProblem<Opti>::dt,
                TrajectoryOptimizationProblem<Opti>::x,
                TrajectoryOptimizationProblem<Opti>::y,
                TrajectoryOptimizationProblem<Opti>::theta,
                HolonomicTrajectoryOptimizationProblem<Opti>::vx,
                HolonomicTrajectoryOptimizationProblem<Opti>::vy,
                HolonomicTrajectoryOptimizationProblem<Opti>::omega,
                HolonomicTrajectoryOptimizationProblem<Opti>::ax,
                HolonomicTrajectoryOptimizationProblem<Opti>::ay,
                HolonomicTrajectoryOptimizationProblem<Opti>::alpha,
                moduleFX, moduleFY);
    } catch (const std::exception& e) {
        throw TrajectoryGenerationException("Error optimizing trajectory: " + std::string(e.what()));
    }
}

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
    netFX.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
    netFY.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
    netTau.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);

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
        indexModuleFX.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
        indexModuleFY.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
        indexModuleTau.reserve(TrajectoryOptimizationProblem<Opti>::sampleTotal);
        for (size_t sampleIndex = 0; sampleIndex < TrajectoryOptimizationProblem<Opti>::sampleTotal; sampleIndex++) {
            ModulePosition modulePosition = SolveModulePosition(TrajectoryOptimizationProblem<Opti>::theta[sampleIndex],
                    SwerveTrajectoryOptimizationProblem::swerveDrivetrain.modules[moduleIndex]);
            indexModuleX.push_back(modulePosition.x);
            indexModuleY.push_back(modulePosition.y);
            indexModuleVX.push_back(HolonomicTrajectoryOptimizationProblem<Opti>::vx[sampleIndex] - indexModuleY[sampleIndex] * HolonomicTrajectoryOptimizationProblem<Opti>::omega[sampleIndex]);
            indexModuleVY.push_back(HolonomicTrajectoryOptimizationProblem<Opti>::vy[sampleIndex] + indexModuleX[sampleIndex] * HolonomicTrajectoryOptimizationProblem<Opti>::omega[sampleIndex]);
            
            indexModuleFX.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
            indexModuleFY.push_back(TrajectoryOptimizationProblem<Opti>::opti.Variable());
            indexModuleTau.push_back(indexModuleX[sampleIndex] * indexModuleFY[sampleIndex]
                                    - indexModuleY[sampleIndex] * indexModuleFX[sampleIndex]);
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

    for (size_t sampleIndex = 0; sampleIndex < TrajectoryOptimizationProblem<Opti>::sampleTotal; sampleIndex++) {
        Expression intervalNetFX = 0;
        Expression intervalNetFY = 0;
        Expression intervalNetTau = 0;
        for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
            intervalNetFX += moduleFX[moduleIndex][sampleIndex];
            intervalNetFY += moduleFY[moduleIndex][sampleIndex];
            intervalNetTau += moduleTau[moduleIndex][sampleIndex];
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
    ModulePosition position{0.0, 0.0};
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
    size_t sampleTotal = ax.size();
    for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
        opti.SubjectTo(netFX[sampleIndex]  == swerveDrivetrain.mass            * ax[sampleIndex]);
        opti.SubjectTo(netFY[sampleIndex]  == swerveDrivetrain.mass            * ay[sampleIndex]);
        opti.SubjectTo(netTau[sampleIndex] == swerveDrivetrain.momentOfInertia * alpha[sampleIndex]);
    }
}

template<typename Opti>
void SwerveTrajectoryOptimizationProblem<Opti>::ApplyPowerConstraints(Opti& opti,
        const std::vector<std::vector<Expression>>& moduleVX, const std::vector<std::vector<Expression>>& moduleVY,
        const std::vector<std::vector<Expression>>& moduleFX, const std::vector<std::vector<Expression>>& moduleFY,
        const SwerveDrivetrain& swerveDrivetrain) {
    size_t moduleCount = swerveDrivetrain.modules.size();
    for (size_t moduleIndex = 0; moduleIndex < moduleCount; moduleIndex++) {
        size_t sampleTotal = moduleVX[moduleIndex].size();
        const SwerveModule& _module = swerveDrivetrain.modules[moduleIndex];
        double maxWheelVelocity = _module.wheelRadius * _module.wheelMaxAngularVelocity;
        double maxForce = _module.wheelMaxTorque / _module.wheelRadius;
        for (size_t sampleIndex = 0; sampleIndex < sampleTotal; sampleIndex++) {
            auto constraint = moduleVX[moduleIndex][sampleIndex] * moduleVX[moduleIndex][sampleIndex]
                            + moduleVY[moduleIndex][sampleIndex] * moduleVY[moduleIndex][sampleIndex]
                            <= maxWheelVelocity * maxWheelVelocity;
            // std::cout << "\n\nMax velocity constraint: " << constraint << std::endl;
            opti.SubjectTo(moduleVX[moduleIndex][sampleIndex] * moduleVX[moduleIndex][sampleIndex]
                            + moduleVY[moduleIndex][sampleIndex] * moduleVY[moduleIndex][sampleIndex]
                            <= maxWheelVelocity * maxWheelVelocity);
            
            opti.SubjectTo(moduleFX[moduleIndex][sampleIndex] * moduleFX[moduleIndex][sampleIndex]
                            + moduleFY[moduleIndex][sampleIndex] * moduleFY[moduleIndex][sampleIndex]
                            <= maxForce * maxForce);
            
            // std::cout << "Applied module " << moduleIndex << " sample " << sampleIndex << " velocity power constraint of " << maxWheelVelocity << std::endl;
        }
    }
}

template<typename Opti>
SwerveSolution SwerveTrajectoryOptimizationProblem<Opti>::
        ConstructSwerveSolution(const Opti& opti,
        const std::vector<Expression>& dt,
        const std::vector<Expression>& x,
        const std::vector<Expression>& y,
        const std::vector<Expression>& theta,
        const std::vector<Expression>& vx,
        const std::vector<Expression>& vy,
        const std::vector<Expression>& omega,
        const std::vector<Expression>& ax,
        const std::vector<Expression>& ay,
        const std::vector<Expression>& alpha,
        const std::vector<std::vector<Expression>>& moduleFX,
        const std::vector<std::vector<Expression>>& moduleFY) {
    return SwerveSolution{
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, dt),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, x),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, y),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, theta),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, vx),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, vy),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, omega),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, ax),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, ay),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, alpha),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, moduleFX),
            TrajectoryOptimizationProblem<Opti>::SolutionValue(opti, moduleFY)};
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
}