#pragma once


namespace helixtrajectory {

    /**
     * @brief This class represents an abstract robot with bumpers. The underlying
     * drivetrain could be differential, swerve, mecanum, omni, etc. This class provides
     * the common obstacle avoidance constraints and mass and moment of inertia fields.
     * 
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
         * @brief Construct a Drivetrain object with the robot's mass, moment of inertia,
         * and bumpers.
         * 
         * @param mass the mass of the entire robot
         * @param momentOfInertia the moment of inertia of the robot about the axis of rotation
         * @param bumpers the bumpers of the robot
         */
        Drivetrain(double mass, double momentOfInertia);

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
         * @brief Destroy the Drivetrain object
         */
        virtual ~Drivetrain() = default;
    };
}