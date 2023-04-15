# Examples with possible new API

## Swerve Drivetrain model

```cpp
SwerveDrivetrain swerveDrivetrain{.mass = 45_kg, .moi = 6_kg_m_sq,
                                  {{+0.6_m, +0.6_m, 0.04_m, 70_rad_per_s, 2_N_m},
                                   {+0.6_m, -0.6_m, 0.04_m, 70_rad_per_s, 2_N_m},
                                   {-0.6_m, +0.6_m, 0.04_m, 70_rad_per_s, 2_N_m},
                                   {-0.6_m, -0.6_m, 0.04_m, 70_rad_per_s, 2_N_m}}};
```

### Example 1: Swerve, one meter forward motion profile

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWaypoint(Pose2d{Translation2d{0_m, 0_m}, 0_deg});
path.PoseWaypoint(Pose2d{Translation2d{1_m, 0_m}, 0_deg});
path.BoundZeroVelocity();
path.CtrlIntCnts({40});
auto solution = TrajectoryOptimizer::Generate(path);
```

### Example 2: Swerve, basic curve

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWaypoint(Pose2d{Translation2d{1_m, 1_m}, Rotation2d{-90_deg}});
path.PoseWaypoint(Pose2d{Translation2d{2_m, 0_m}, Rotation2d{0_deg}});
path.BoundZeroVelocity();
path.CtrlIntCnts({40});
auto solution = TrajectoryOptimizer::Generate(path);
```

### Example 3: Swerve, three waypoints

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWaypoint(Pose2d{Translation2d{0_m, 0_m}, Rotation2d{90_deg}});
path.PoseWaypoint(Pose2d{Translation2d{1_m, 1_m}, Rotation2d{0_deg}});
path.PoseWaypoint(Pose2d{Translation2d{2_m, 0_m}, Rotation2d{90_deg}});
path.BoundZeroVelocity();
path.CtrlIntCnts({40, 40});
auto solution = TrajectoryOptimizer::Generate(path);
```

### Example 4: Swerve, ending velocity

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWaypoint(Pose2d{Translation2d{0_m, 0_m}, Rotation2d{0_deg}});
path.PoseWaypoint(Pose2d{Translation2d{0_m, 1_m}, Rotation2d{0_deg}});
path.WaypointVelocity(0.2_mps, 0.0_mps);
// path.WaypointVelocityMagnitude(0.2_mps); // <--- alternative formulation
// path.WaypointVelocityDirection(0_deg);   // <---------------------------
path.InitialZeroVelocity();
path.CtrlIntCnts({40});
auto solution = TrajectoryOptimizer::Generate(path);
```

### Example 5: Swerve, circle obstacle

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWaypoint(Pose2d{Translation2d{0_m, 0_m}, Rotation2d{0_deg}});
path.SegmentObstacle(CircleObstacle(0.5_m, 0.5_m, 0.1_m));
path.PoseWaypoint(Pose2d{Translation2d{1_m, 0_m}, Rotation2d{0_deg}});
path.BoundZeroVelocity();
path.CtrlIntCnts({40});
auto solution = TrajectoryOptimizer::Generate(path);
```

### Example 6: Approach a pick up station at a certain direction

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);

// starting position
path.PoseWaypoint(Pose2d{Translation2d{0_m, 0_m}, Rotation2d{0_deg}});

// align towards the station one meter behind
path.PoseWaypoint(Pose2d{Translation2d{1_m, 1_m}, Rotation2d{90_deg}});
path.WaypointZeroAngularVelocity();
path.WaypointVelocityDirection(90_deg);

// go up to the station
// In practice, the optimizer will still end up aligning the heading without
// the pose constraint since it's most optimal
path.TranslationWaypoint(Translation2d{1_m, 2_m});

// realign behind the station
path.PoseWaypoint(Pose2d{Translation2d{1_m, 1_m}, Rotation2d{90_deg}});
path.WaypointZeroAngularVelocity();
path.WaypointVelocityDirection(90_deg);

// ending position
path.PoseWaypoint(Pose2d{Translation2d{2_m, 0_m}, Rotation2d{180_deg}});

path.BoundZeroVelocity();
path.CtrlIntCnts({40, 30, 30, 40});
auto solution = TrajectoryOptimizer::Generate(path);
```

### Example 7: Circular path with a point-point constraint

Note that forcing a circular path is not a common problem in FRC, this example
is only to demonstrate how various constraints work.

```cpp
SwervePath path;
path.SetDrivetrain(swerveDrivetrain);

path.PoseWaypoint(Pose2d{Translation2d{0_m, 0_m}, Rotation2d{0_deg}});
path.SegmentPointPointConstraint(
    Point2d{0_m, 0_m} // robot point--center of bot
    Point2d{1_m, 0_m} // field point to orbit around
    1_m); // stay a distance of 1_m away to force circular motion

// tell optimizer to go in +y direction rather than -y
path.AddInitialGuess(Pose2d{Translation2d{0_m, 0_m}, Rotation2d{0_deg}});

path.PoseWaypoint(Pose2d{Translation2d{2_m, 0_m}, Rotation2d{0_deg}});

path.BoundZeroVelocity();
path.CtrlIntCnts({30});
auto solution = TrajectoryOptimizer::Generate(path);
```
