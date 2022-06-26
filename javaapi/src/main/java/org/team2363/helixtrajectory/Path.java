package org.team2363.helixtrajectory;

public class Path {

    final Waypoint[] waypoints;

    public Path(Waypoint... waypoints) {
        this.waypoints = waypoints;
    }

    public int length() {
        return waypoints.length;
    }

    public Waypoint get(int index) throws IndexOutOfBoundsException {
        if (index >= 0 && index < length()) {
            return waypoints[index];
        } else {
            throw new IndexOutOfBoundsException();
        }
    }

    double[] asArray() {
        double[] array = new double[length() * Waypoint.ARRAY_LENGTH];
        for (int i = 0; i < length(); i++) {
            array[Waypoint.ARRAY_LENGTH*i    ] = get(i).x;
            array[Waypoint.ARRAY_LENGTH*i + 1] = get(i).y;
            array[Waypoint.ARRAY_LENGTH*i + 2] = get(i).heading;
        }
        return array;
    }
}
