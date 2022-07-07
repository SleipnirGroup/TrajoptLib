package org.team2363.helixtrajectory;

public class Obstacle {

    public final double safetyDistance;
    final ObstaclePoint[] obstaclePoints;

    public Obstacle(double safetyDistance, ObstaclePoint... obstaclePoints) {
        this.safetyDistance = safetyDistance;
        this.obstaclePoints = obstaclePoints;
    }

    public int length() {
        return obstaclePoints.length;
    }

    public ObstaclePoint get(int index) throws IndexOutOfBoundsException {
        if (index >= 0 && index < length()) {
            return obstaclePoints[index];
        } else {
            throw new IndexOutOfBoundsException();
        }
    }
}
