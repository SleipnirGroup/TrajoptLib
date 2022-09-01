package org.team2363.helixtrajectory;

import java.util.Objects;

public final class OptimalTrajectoryGenerator {

    private static boolean isPluginLoaded = false;

    private static void loadPlugin() throws PluginLoadException {
        if (!isPluginLoaded) {
            try {
                System.loadLibrary("helixtrajectory");
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

    private native HolonomicTrajectory generateHolonomicTrajectory(HolonomicDrivetrain holonomicDrivetrain,
            HolonomicPath holonomicPath) throws InvalidPathException, TrajectoryGenerationException;

    public static HolonomicTrajectory generate(HolonomicDrivetrain holonomicDrivetrain, HolonomicPath holonomicPath)
            throws NullPointerException, InvalidPathException, PluginLoadException, TrajectoryGenerationException {
        return new OptimalTrajectoryGenerator().generateHolonomicTrajectory(
                Objects.requireNonNull(holonomicDrivetrain, "Holonomic Trajectory Generator holonomic drive cannot be null"),
                Objects.requireNonNull(holonomicPath, "Holonomic Trajectory Generator holonomic path cannot be null"));
    }
}