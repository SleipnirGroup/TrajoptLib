#pragma once

#include <tuple>

#include <casadi/casadi.hpp>

namespace helixtrajectory {
    class SwerveDrive {
    public:
        const double wheelbaseX, wheelbaseY, length, width, mass, moi,
                omegaMax, tauMax, wheelRadius;

        SwerveDrive(const double wheelbaseX, const double wheelbaseY,
                const double length, const double width, const double mass,
                const double moi, const double omegaMax, const double tauMax,
                const double wheelRadius);

        void ApplyKinematicsConstraints(casadi::Opti& solver,
                const casadi::MX& theta, const casadi::MX& vx, const casadi::MX& vy,
                const casadi::MX& omega, const casadi::MX& ax, const casadi::MX& ay,
                const casadi::MX& alpha, const size_t nTotal) const;

    private:
        const double maxWheelVelocitySquared, maxForceSquared, moduleAngle, diagonal;

        const casadi::MX SolveModulePositions(const casadi::MX& theta) const;
    };
}