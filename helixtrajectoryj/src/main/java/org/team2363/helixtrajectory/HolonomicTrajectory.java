package org.team2363.helixtrajectory;

import java.util.List;

public final class HolonomicTrajectory extends Trajectory {

    public final List<HolonomicTrajectorySample> holonomicSamples;

    private HolonomicTrajectory(List<HolonomicTrajectorySample> holonomicSamples) {
        super(holonomicSamples);
        this.holonomicSamples = holonomicSamples;
    }

    public HolonomicTrajectory(HolonomicTrajectorySample... holonomicSamples) throws NullPointerException {
        this(List.of(holonomicSamples));
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("[\n");
        for (HolonomicTrajectorySample holonomicSample : holonomicSamples) {
            builder.append("    {\n");
            builder.append("        \"timestamp\": ").append(holonomicSample.timestamp).append(",\n");
            builder.append("        \"x\": ").append(holonomicSample.x).append(",\n");
            builder.append("        \"y\": ").append(holonomicSample.y).append(",\n");
            builder.append("        \"heading\": ").append(holonomicSample.heading).append(",\n");
            builder.append("        \"velocity_x\": ").append(holonomicSample.velocityX).append(",\n");
            builder.append("        \"velocity_y\": ").append(holonomicSample.velocityY).append(",\n");
            builder.append("        \"angular_velocity\": ").append(holonomicSample.angularVelocity).append("\n");
            builder.append("    },\n");
        }
        builder.deleteCharAt(builder.length() - 2);
        builder.append("]");
        return builder.toString();
    }
}
