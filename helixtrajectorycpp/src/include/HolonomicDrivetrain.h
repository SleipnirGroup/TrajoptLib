#pragma once

#include "Drivetrain.h"

namespace helixtrajectory {

    /**
     * @brief This class represents a type of drivetrain that is holonomic. Holonomic drivetrains have
     * complete (or approximate) control over the three degrees of freedom: position and rotation.
     * For example, mecanum drivetrains and swerve drivetrains can both manipulate their motors
     * to have any overall velocity vector while having any heading. Holonomic Drivetrain introduces
     * no new fields, but this inheritance structure is still used since it is logical and maintains
     * consistency with the rest of the api.
     * 
     * @author Justin Babilino
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
        HolonomicDrivetrain(double mass, double momentOfInertia);
    };
}