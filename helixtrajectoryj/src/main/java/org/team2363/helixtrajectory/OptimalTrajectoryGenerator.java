package org.team2363.helixtrajectory;

import java.util.Objects;

import org.team2363.util.RuntimeLoader;

public final class OptimalTrajectoryGenerator {

    private static boolean isPluginLoaded = false;

    private static void loadPlugin() throws PluginLoadException {
        if (!isPluginLoaded) {
            try {
                var loader = new RuntimeLoader<OptimalTrajectoryGenerator>(
                        "helixtrajectory", null, OptimalTrajectoryGenerator.class);
                loader.loadLibrary();
                isPluginLoaded = true;
            } catch (SecurityException e) {
                throw new PluginLoadException("Could not load HelixTrajectory: " + e.getMessage());
            } catch (UnsatisfiedLinkError e) {
                throw new PluginLoadException("Could not load HelixTrajectory: " + e.getMessage());
            }
        }
    }

    private OptimalTrajectoryGenerator() throws PluginLoadException {
        loadPlugin();
    }

    private native HolonomicTrajectory generateHolonomicTrajectory(SwerveDrivetrain swerveDrivetrain,
            HolonomicPath holonomicPath) throws InvalidPathException, TrajectoryGenerationException;

    public static HolonomicTrajectory generate(SwerveDrivetrain swerveDrivetrain, HolonomicPath holonomicPath)
            throws NullPointerException, InvalidPathException, PluginLoadException, TrajectoryGenerationException {

        return new OptimalTrajectoryGenerator().generateHolonomicTrajectory(
                Objects.requireNonNull(swerveDrivetrain, "Holonomic Trajectory Generator swerve drivetrain cannot be null"),
                Objects.requireNonNull(holonomicPath, "Holonomic Trajectory Generator holonomic path cannot be null"));
    }
}