// Copyright (c) TrajoptLib contributors

package org.sleipnirgroup.trajoptlib;

import java.util.List;

public abstract class Trajectory {
    public final List<? extends TrajectorySegment> segments;

    protected Trajectory(List<? extends TrajectorySegment> segments) {
        this.segments = segments;
    }
}
