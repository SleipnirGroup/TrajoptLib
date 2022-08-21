package org.team2363.helixtrajectory;

import java.util.Objects;

public final class OptimalHolonomicTrajectoryGenerator implements HolonomicTrajectoryGenerator {

    private static boolean isPluginLoaded = false;

    private static void loadPlugin() throws PluginLoadException {
        if (!isPluginLoaded) {
            try {
                System.loadLibrary("helixtrajectory");
                isPluginLoaded = true;
            } catch (SecurityException error) {
                throw new PluginLoadException("Could not load HelixTrajectory: " + error.getMessage());
            } catch (UnsatisfiedLinkError error) {
                throw new PluginLoadException("Could not load HelixTrajectory: " + error.getMessage());
            }
        }
    }

    private final HolonomicDrivetrain holonomicDrive;
    private final HolonomicPath holonomicPath;

    public OptimalHolonomicTrajectoryGenerator(HolonomicDrivetrain holonomicDrive,
            HolonomicPath holonomicPath) throws NullPointerException, PluginLoadException {
        loadPlugin();
        this.holonomicDrive = Objects.requireNonNull(holonomicDrive, "Holonomic Trajectory Generator holonomic drive cannot be null");
        this.holonomicPath = Objects.requireNonNull(holonomicPath, "Holonomic Trajectory Generator holonomic path cannot be null");
    }

    public native HolonomicTrajectory generate() throws InvalidPathException, TrajectoryGenerationException;
    // it's ok to make this public since it's not accesible unless the contructor is able to load the plugin
}