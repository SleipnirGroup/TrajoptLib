package org.team2363.helixtrajectory;

public record HolonomicTrajectorySample (double timestamp, double x, double y, double heading, double velocityX, double velocityY, double angularVelocity) {
}