package org.team2363.helixtrajectory;

import java.util.List;

public class HolonomicTrajectory {

    public final List<HolonomicTrajectorySample> samples;

    public HolonomicTrajectory(HolonomicTrajectorySample... samples) {
        this.samples = List.of(samples);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("[\n");
        for (HolonomicTrajectorySample sample : samples) {
            builder.append("    {\n");
            builder.append("        \"ts\": ").append(sample.timestamp()).append(",\n");
            builder.append("        \"x\": ").append(sample.x()).append(",\n");
            builder.append("        \"y\": ").append(sample.y()).append(",\n");
            builder.append("        \"heading\": ").append(sample.heading()).append(",\n");
            builder.append("        \"vx\": ").append(sample.velocityX()).append(",\n");
            builder.append("        \"vy\": ").append(sample.velocityY()).append(",\n");
            builder.append("        \"omega\": ").append(sample.angularVelocity()).append("\n");
            builder.append("    },\n");
        }
        builder.deleteCharAt(builder.length() - 2);
        builder.append("]");
        return builder.toString();
    }
}
