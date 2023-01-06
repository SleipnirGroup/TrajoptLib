// Copyright (c) TrajoptLib contributors

package org.team2363.helixtrajectory;

import java.io.IOException;
import java.util.Objects;

import org.team2363.util.DependencyExtractor;
import org.team2363.util.RuntimeLoader;

public final class OptimalTrajectoryGenerator {
    private static boolean isPluginLoaded = false;

    private static void loadPlugin() throws PluginLoadException {
        if (!isPluginLoaded) {
            try {
                String osName = System.getProperty("os.name");
                if (osName.startsWith("Windows")) {
                    for (String libName : new String[]{"libcasadi.dll", "libstdc++-6.dll", "libcasadi_nlpsol_ipopt.dll", "libgfortran-3.dll", "libquadmath-0.dll", "libgcc_s_seh-1.dll"}) {
                        new DependencyExtractor<OptimalTrajectoryGenerator>(
                                libName, RuntimeLoader.getDefaultExtractionRoot(), OptimalTrajectoryGenerator.class).loadLibrary();
                    }
                } else if (osName.startsWith("Mac")) {
                    for (String libName : new String[]{"libcasadi.dylib", "libc++.1.dylib", "libcasadi_nlpsol_ipopt.dylib", "libgfortran.4.dylib", "libquadmath.0.dylib", "libgcc_s.1.dylib"}) {
                        new DependencyExtractor<OptimalTrajectoryGenerator>(
                                libName, RuntimeLoader.getDefaultExtractionRoot(), OptimalTrajectoryGenerator.class).loadLibrary();
                    }
                } else if (osName.startsWith("Linux")) {
                    for (String libName : new String[]{"libcasadi.so", "libcasadi_nlpsol_ipopt.so", "libgfortran-ed201abd.so.3.0.0"}) {
                        new DependencyExtractor<OptimalTrajectoryGenerator>(
                                libName, RuntimeLoader.getDefaultExtractionRoot(), OptimalTrajectoryGenerator.class).loadLibrary();
                    }
                } else {
                    throw new PluginLoadException("Unsupported operating system", null);
                }
                new RuntimeLoader<OptimalTrajectoryGenerator>(
                        "helixtrajectory", RuntimeLoader.getDefaultExtractionRoot(), OptimalTrajectoryGenerator.class).loadLibrary();
                isPluginLoaded = true;
            } catch (IOException ioe) {
                throw new PluginLoadException("Could not load HelixTrajectory: " + ioe.getMessage(), ioe);
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
