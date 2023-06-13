// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import java.util.Objects;

import org.sleipnirgroup.util.PluginLoader;

public final class OptimalTrajectoryGenerator {

    public static void testFunc() throws Exception {
        PluginLoader.loadPlugin();
    }

    private native HolonomicTrajectory generateHolonomicTrajectory(SwerveDrivetrain swerveDrivetrain,
            HolonomicPath holonomicPath) throws InvalidPathException, TrajectoryGenerationException;

    public static HolonomicTrajectory generate(SwerveDrivetrain swerveDrivetrain, HolonomicPath holonomicPath)
            throws NullPointerException, InvalidPathException, PluginLoadException, TrajectoryGenerationException {
        PluginLoader.loadPlugin();
        return new OptimalTrajectoryGenerator().generateHolonomicTrajectory(
                Objects.requireNonNull(swerveDrivetrain, "Holonomic Trajectory Generator swerve drivetrain cannot be null"),
                Objects.requireNonNull(holonomicPath, "Holonomic Trajectory Generator holonomic path cannot be null"));
    }
}
