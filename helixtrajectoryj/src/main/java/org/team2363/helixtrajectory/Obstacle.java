package org.team2363.helixtrajectory;

import static org.team2363.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class Obstacle {

    public final double safetyDistance;
    public final boolean applyToAllSegments;
    public final List<? extends ObstaclePoint> points;

    public Obstacle(double safetyDistance, boolean applyToAllSegments,
            List<? extends ObstaclePoint> points) throws NullPointerException {
        this.safetyDistance = safetyDistance;
        this.applyToAllSegments = applyToAllSegments;
        this.points = requireNonNullAndWrapUnmodifiable(points, "List of obstacle points cannot be null", "Obstacle point cannot be null");
    }

    @Override
    public String toString() {
        return "{\"safety_distance\": " + safetyDistance
            + ", \"apply_to_all_segments\": " + applyToAllSegments
            + ", \"points\": " + points
            + "}";
    }
}