#pragma once

// #include <array>
// #include <cmath>
#include <optional>
#include <variant>

#include "constraint/HeadingConstraint.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

/**
 * @brief Enumerates the various coordinate systems used in trajectory optimization.
 * The field coordinate system is the base system that is fixed to the field with
 * some arbitrary center. Other coordinate systems defined do not have a relative
 * velocity to the field coordinate system, but their position and orientation depends
 * on the current position of the robot. For example, if the robot is spinning,
 * then the robot has angular velocity relative to all the systems defined here, and
 * the magnitude of the robot's velocity is the same in all the systems but the
 * direction varies.
 * 
 * We define these other systems: the robot coordinate system, the
 * robot nonrotating coordinate system, and the robot velocity coordinate system.
 * The robot coordinate system is centered on and oriented with the front of the
 * robot towards the x-axis. The robot nonrotating coordinate system is centered
 * on the robot but is oriented with the field. The robot velocity coordinate system
 * is centered on the robot and it is oriented with the robot's velocity in the
 * x-direction.
 * 
 * Note that in differential drivetrains, the robot coordinate system
 * is equivalent ot the robot velocity coordinate system.
 */
enum class CoordinateSystem {
    /**
     * @brief the coordinate system of the field
     */
    kField,
    /**
     * @brief the coordinate system of the robot
     */
    kRobot,
};

// inline std::array<double, 2> ApplyRotation(std::array<double, 2> vect, double heading) {
//     if (vect[0] == 0.0 && vect[1] == 0.0) {
//         return {0.0, 0.0};
//     } else {
//         auto cosHeading = cos(heading);
//         auto sinHeading = sin(heading);
//         return {cosHeading * vect[0] - sinHeading * vect[1], sinHeading * vect[0] + cosHeading * vect[1]};
//     }
// }

using ConstraintVariant = std::variant<TranslationConstraint, HeadingConstraint, PoseConstraint, ObstacleConstraint>;

class Constraint {
public:
    std::optional<SolutionError> CheckState(double x, double y,
            double heading, const SolutionTolerances& tolerances) const noexcept;

    bool IsTranslationConstraint() const;
    bool IsHeadingConstraint() const;
    bool IsPoseConstraint() const;
    bool IsObstacleConstraint() const;

    const TranslationConstraint& GetTranslationConstraint() const;
    TranslationConstraint& GetTranslationConstraint();

    const HeadingConstraint& GetHeadingConstraint() const;
    HeadingConstraint& GetHeadingConstraint();

    const PoseConstraint& GetPoseConstraint() const;
    PoseConstraint& GetPoseConstraint();

    const ObstacleConstraint& GetObstacleConstraint() const;
    ObstacleConstraint& GetObstacleConstraint();

    Constraint(const TranslationConstraint& translationConstraint);
    Constraint(const HeadingConstraint& headingConstraint);
    Constraint(const PoseConstraint& poseConstraint);
    Constraint(const ObstacleConstraint& obstacleConstraint);

private:
    ConstraintVariant constraint;
};
}

template<>
struct fmt::formatter<helixtrajectory::Constraint> {

    template<typename ParseContext>
    constexpr auto parse(ParseContext& ctx);

    template<typename FormatContext>
    auto format(const helixtrajectory::Constraint& constraint, FormatContext& ctx);
};