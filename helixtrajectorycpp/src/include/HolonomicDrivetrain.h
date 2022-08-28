#pragma once

#include <casadi/casadi.hpp>

#include "Drivetrain.h"
#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief Represents a type of drivetrain that is holonomic. Holonomic drivetrains allow
     * the robot to have complete (or approximate) control over the three degrees of freedom:
     * position and rotation. For example, mecanum drivetrains and swerve drivetrains can both
     * manipulate their motors to have any overall velocity vector while having any heading.
     * HolonomicDrivetrain introduces no new fields, but it forces subtypes to implement the
     * kinematics constraints method, which ties the robot's overall velocity variables to the
     * actuators on the drivetrain.
     */
    class HolonomicDrivetrain : public Drivetrain {
    protected:
        /**
         * @brief Construct a new HolonomicDrivetrain with the robot's mass, moment of inertia, and bumpers.
         * 
         * @param mass the mass of the entire robot
         * @param momentOfInertia the moment of inertia of the robot about the center of rotation, which 
         * @param bumpers the bumpers of the robot represented as an obstacle
         */
        HolonomicDrivetrain(double mass, double momentOfInertia, const Obstacle& bumpers);

    public:
        
    };
}