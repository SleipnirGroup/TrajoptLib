// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import static org.sleipnirgroup.util.ObjectChecker.requireNonNullAndWrapUnmodifiable;

import java.util.List;

public final class Obstacle {
    public final double safetyDistance;
    public final List<? extends ObstaclePoint> points;

    public Obstacle(double safetyDistance, boolean applyToAllSegments,
            List<? extends ObstaclePoint> points) throws NullPointerException {
        this.safetyDistance = safetyDistance;
        this.points = requireNonNullAndWrapUnmodifiable(points, "List of obstacle points cannot be null", "Obstacle point cannot be null");
    }

    @Override
    public String toString() {
        return "{\"safety_distance\": " + safetyDistance
            + ", \"points\": " + points
            + "}";
    }
}
