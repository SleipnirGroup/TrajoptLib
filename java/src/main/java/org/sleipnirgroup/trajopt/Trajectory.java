// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajopt;

import java.util.List;

public abstract class Trajectory {
    public final List<? extends TrajectorySample> samples;

    protected Trajectory(List<? extends TrajectorySample> samples) {
        this.samples = samples;
    }
}
