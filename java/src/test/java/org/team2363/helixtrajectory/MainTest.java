// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajoptlib;

import java.util.List;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

class MainTest {
    @Test void generateTrajectory() {
        SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(45, 6,
            List.of(new SwerveModule(+0.6, +0.6, 0.04, 70, 2),
             new SwerveModule(+0.6, -0.6, 0.04, 70, 2),
             new SwerveModule(-0.6, +0.6, 0.04, 70, 2),
             new SwerveModule(-0.6, -0.6, 0.04, 70, 2)),
            new Obstacle(0, true, List.of(
             new ObstaclePoint(+0.5, +0.5),
             new ObstaclePoint(-0.5, +0.5),
             new ObstaclePoint(-0.5, -0.5),
             new ObstaclePoint(+0.5, -0.5))));

        HolonomicPath holonomicPath = new HolonomicPath(List.of(
            new HolonomicWaypoint( 4,  0,    0, 0, 0, 0, true, true, true,  true,  true,  true,  true,    0, List.of(), List.of()),
            new HolonomicWaypoint( 0,  4, 1.57, 0, 0, 0, true, true, true, false, false, false, false,  10, List.of(), List.of()),
            new HolonomicWaypoint(-4,  0,    0, 0, 0, 0, true, true, true, false, false, false, false,  10, List.of(), List.of()),
            new HolonomicWaypoint( 0, -4, 3.14, 0, 0, 0, true, true, true, false, false, false, false,  10, List.of(), List.of()),
            new HolonomicWaypoint( 4,  0, 4.71, 0, 0, 0, true, true, true,  true,  true,  true,  true,  10, List.of(), List.of())
        ));

        System.out.println("Drivetrain:\n");
        System.out.println(swerveDrivetrain);
        System.out.println("Path:\n");
        System.out.println(holonomicPath);

        assertDoesNotThrow(() -> {
            HolonomicTrajectory holonomicTrajectory = OptimalTrajectoryGenerator.generate(swerveDrivetrain, holonomicPath);
            System.out.println("Trajectory:\n");
            System.out.println(holonomicTrajectory);
        });
    }
}
