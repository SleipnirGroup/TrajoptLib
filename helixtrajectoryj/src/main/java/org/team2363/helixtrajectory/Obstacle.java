package org.team2363.helixtrajectory;

import static org.team2363.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class Obstacle {

    public final double safetyDistance;
    public final boolean applyToAllSegments;
    public final List<? extends ObstaclePoint> obstaclePoints;

    public Obstacle(double safetyDistance, boolean applyToAllSegments,
            List<? extends ObstaclePoint> obstaclePoints) throws NullPointerException {
        this.safetyDistance = safetyDistance;
        this.applyToAllSegments = applyToAllSegments;
        this.obstaclePoints = requireNonNullAndWrapUnmodifiable(obstaclePoints, "List of obstacle points cannot be null", "Obstacle point cannot be null");
    }
}