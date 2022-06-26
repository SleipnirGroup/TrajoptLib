package org.team2363.helixtrajectory;

public class Main {

    public static void main(String[] args) {
        SwerveDrive drive = new SwerveDrive(0.622, 0.572, 0.954, 0.903, 46.7, 5.6, 70, 1.9, 0.051);
        Path path = new Path(
                new Waypoint(0.0, 0.0, Math.PI / 2),
                new Waypoint(2.0, 2.0, 0.0),
                new Waypoint(4.0, 0.0, -Math.PI / 2));
        TrajectoryGenerator generator = new TrajectoryGenerator(drive);
        System.out.print(generator.generate(path));
    }
}
