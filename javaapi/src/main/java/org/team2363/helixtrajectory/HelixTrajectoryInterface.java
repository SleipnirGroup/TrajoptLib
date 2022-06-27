package org.team2363.helixtrajectory;

public class HelixTrajectoryInterface {

    static {
        // System.loadLibrary("helixtrajectorycpp");
        // System.load("/Users/jlbabilino/Documents/TripleHelix/Programming/Repositories/HelixTrajectoryJ/casadi/libcasadi.dylib");
        // System.load("/Users/jlbabilino/Documents/TripleHelix/Programming/Repositories/HelixTrajectoryJ/casadi/libcasadi_nlpsol_ipopt.dylib");
        // System.load("/Users/jlbabilino/Documents/TripleHelix/Programming/Repositories/HelixTrajectoryJ/helixtrajectorycpp/build/lib/main/debug/libhelixtrajectorycpp.dylib");
        System.loadLibrary("helixtrajectorycpp");
    }

    public native double[] generateTrajectory(double[] robotConfiguration, double[] path);
}