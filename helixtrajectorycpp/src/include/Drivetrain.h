#pragma once

#include "Obstacle.h"

namespace helixtrajectory {

    /**
     * @brief This class represents an abstract robot with bumpers. The underlying robot
     * drivetrain could be differential, swerve, mecanum, omni, etc. This class provides
     * the common obstacle avoidance constraints and mass and moment of inertia fields.
     * Related to the 2D coordinate system of the field called the field coordinate system,
     * the robot coordinate system is defined with its center as the center of the robot,
     * and it rotates as the robot rotates. The nonrotating robot coordinate system is defined
     * with the same center as the robot coordinate system, but it does not rotate and its axes
     * always point in the same directions as the field coordinate system. The robot rotates
     * relative to the nonrotating robot coordinate system.
     * 
     * The axis of rotation of the robot pierces through the origin of the robot coordinate
     * system and is normal to the plane of the field.
     * 
     * Bumpers are represented as an obstacle. A bumper corner is analogous to an obstacle point
     * on the bumpers. The bumper corner points are specified relative to the robot coordinate
     * system. Using an obstacle as the robot's bumpers is very powerful, as it is possible to create
     * bumpers with rounded corners, circular bumpers, rectanglar bumpers with two semi circles
     * on side, or many other shapes.
     * 
     * @author Justin Babilino
     */
    class Drivetrain {
    protected:
        /**
         * @brief Construct a new Drive object with the robot's mass, moment of inertia,
         * and bumpers. Bumpers are represented as an obstacle.
         * 
         * @param mass the mass of the entire robot
         * @param momentOfInertia the moment of inertia of the robot about the center of rotation
         * @param bumpers the bumpers of the robot
         */
        Drivetrain(double mass, double momentOfInertia, const Obstacle& bumpers);

    public:
        /**
         * @brief mass of the robot
         */
        double mass;
        /**
         * @brief moment of inertia of robot about axis of rotation, through
         * center of robot coordinate system
         */
        double momentOfInertia;
        /**
         * @brief the boundaries of the robot's bumpers, represented as an Obstacle.
         */
        Obstacle bumpers;

        /**
         * @brief Destroy the Drivetrain object
         */
        virtual ~Drivetrain() = default;
    };
}