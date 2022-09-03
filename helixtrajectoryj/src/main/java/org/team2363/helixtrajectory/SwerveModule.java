package org.team2363.helixtrajectory;


public final class SwerveModule {

    public final double x;
    public final double y;
    public final double wheelRadius;
    public final double wheelMaxAngularVelocity;
    public final double wheelMaxTorque;

    public SwerveModule(double x, double y, double wheelRadius, double wheelMaxAngularVelocity, double wheelMaxTorque) {
        this.x = x;
        this.y = y;
        this.wheelRadius = wheelRadius;
        this.wheelMaxAngularVelocity = wheelMaxAngularVelocity;
        this.wheelMaxTorque = wheelMaxTorque;
    }

    @Override
    public String toString() {
        return "{\"x\": " + x
            + ", \"y\": " + y
            + ", \"wheel_radius\": " + wheelRadius
            + ", \"wheel_max_angular_velocity\": " + wheelMaxAngularVelocity
            + ", \"wheel_max_torque\": " + wheelMaxTorque
            + "}";
    }
}