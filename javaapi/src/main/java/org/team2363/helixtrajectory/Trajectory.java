package org.team2363.helixtrajectory;

public class Trajectory {

    final TrajectorySample[] samples;

    public Trajectory(TrajectorySample... samples) {
        this.samples = samples;
    }

    public int length() {
        return samples.length;
    }

    public TrajectorySample get(int index) throws IndexOutOfBoundsException {
        if (index >= 0 && index < length()) {
            return samples[index];
        } else {
            throw new IndexOutOfBoundsException();
        }
    }

    static Trajectory fromArray(double[] array) {
        int sampleCount = array.length / TrajectorySample.ARRAY_LENGTH;
        TrajectorySample[] samples = new TrajectorySample[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            samples[i] = new TrajectorySample(
                array[TrajectorySample.ARRAY_LENGTH * i    ],
                array[TrajectorySample.ARRAY_LENGTH * i + 1],
                array[TrajectorySample.ARRAY_LENGTH * i + 2],
                array[TrajectorySample.ARRAY_LENGTH * i + 3],
                array[TrajectorySample.ARRAY_LENGTH * i + 4],
                array[TrajectorySample.ARRAY_LENGTH * i + 5],
                array[TrajectorySample.ARRAY_LENGTH * i + 6]);
        }
        return new Trajectory(samples);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        builder.append("[\n");
        for (TrajectorySample sample : samples) {
            builder.append("    {\n");
            builder.append("        \"ts\": ").append(sample.ts).append(",\n");
            builder.append("        \"x\": ").append(sample.x).append(",\n");
            builder.append("        \"y\": ").append(sample.y).append(",\n");
            builder.append("        \"heading\": ").append(sample.heading).append(",\n");
            builder.append("        \"vx\": ").append(sample.vx).append(",\n");
            builder.append("        \"vy\": ").append(sample.vy).append(",\n");
            builder.append("        \"omega\": ").append(sample.omega).append("\n");
            builder.append("    },\n");
        }
        builder.deleteCharAt(builder.length() - 2);
        builder.append("]");
        return builder.toString();
    }
}
