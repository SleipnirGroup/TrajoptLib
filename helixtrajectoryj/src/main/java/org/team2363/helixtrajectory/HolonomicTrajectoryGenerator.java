package org.team2363.helixtrajectory;

public interface HolonomicTrajectoryGenerator {
    
    HolonomicTrajectory generate() throws InvalidPathException, TrajectoryGenerationException;
}