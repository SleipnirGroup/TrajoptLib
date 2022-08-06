package org.team2363.helixtrajectory;

import java.util.Collections;
import java.util.List;

public final class Obstacle {

    public final double safetyDistance;
    public final List<ObstaclePoint> obstaclePoints;

    public Obstacle(double safetyDistance, ObstaclePoint... obstaclePoints) throws NullPointerException {
        this.safetyDistance = safetyDistance;
        this.obstaclePoints = Collections.unmodifiableList(List.of(obstaclePoints));
    }
}