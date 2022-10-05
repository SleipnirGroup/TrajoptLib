#pragma once

#include "PlanarBound.h"
#include "ScalarBound.h"

namespace helixtrajectory {

    /**
     * @brief This class represents a constraint on a 2D special euclidian (SE2) space
     * variable such as position, velocity, or acceleration. The translational component
     * of the SE2 is the vector of position, velocity, or acceleration, and the rotational
     * component is the scalar of heading, angular velocity, or angular acceleration.
     * Constraints are applied to the translational component by bounding the 2D vector
     * within a region defined by a PlanarBound.
     * 
     * @author Justin Babilino
     */
    class SE2Constraint {
    public:
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
            /**
             * @brief the coordinate system centered on the robot but oriented with the field
             */
            kRobotNonrotating,
            /**
             * @brief the coordinate system centered on the robot but oriented with the robot's velocity
             */
            kRobotVelocity
        };
        /**
         * @brief the translational component of the SE2 constraint
         */
        PlanarBound translationBound;
        /**
         * @brief the rotational component of the SE2 constraint
         */
        ScalarBound rotationBound;
        /**
         * @brief the coordinate system constraints are made with respect to
         */
        CoordinateSystem coordinateSystem;

        /**
         * @brief Construct a SE2Constraint with a translation bound, rotation bound, and
         * coordinate system.
         * 
         * @param translationBound the translational component of the SE2 constraint
         * @param rotationBound the rotational component of the SE2 constraint
         * @param robotRelative the coordinate system constraints are made with respect to
         */
        SE2Constraint(const PlanarBound& translationBound, const ScalarBound& rotationBound, CoordinateSystem coordinateSystem);

        /**
         * @brief Check if the constraint is valid. An SE2 constraint is valid if its translation
         * bound and rotation bound are valid.
         * 
         * @return true if and only if this SE2 constraint is valid 
         */
        bool IsValid() const noexcept;
    };
}