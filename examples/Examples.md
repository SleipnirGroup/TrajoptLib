# Examples with possible new API

`SwervePathBuilderBuilder` is the class used to build paths that are optimized into full trajectories.
`Wpt` is the abbreviation for waypoint, an instantaneous moment in the path where certain constrains on the
robot's state are applied, and `Sgmt` is the abbreviation for segments, the continuum of state between waypoints where
constraints can also be applied.

## Swerve Drivetrain model

```cpp
SwerveDrivetrain swerveDrivetrain{.mass = 45, .moi = 6,
                                  {{+0.6, +0.6, 0.04, 70, 2},
                                   {+0.6, -0.6, 0.04, 70, 2},
                                   {-0.6, +0.6, 0.04, 70, 2},
                                   {-0.6, -0.6, 0.04, 70, 2}}};
```

### Example 1: Swerve, one meter forward motion profile

```cpp
SwervePathBuilderBuilder path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWpt(0, 0.0, 0.0, 0.0);
path.PoseWpt(1, 1.0, 0.0, 0.0);
path.WptZeroVelocity(0);
path.WptZeroVelocity(1);
path.ControlIntervalCounts({40});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```

### Example 2: Swerve, basic curve

```cpp
SwervePathBuilderBuilder path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWpt(0, 1.0, 1.0, -PI/2);
path.PoseWpt(1, 2.0, 0.0, 0.0);
path.WptZeroVelocity(0);
path.WptZeroVelocity(1);
path.ControlIntervalCounts({40});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```

### Example 3: Swerve, three waypoints

```cpp
SwervePathBuilder path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWpt(0, 0.0, 0.0, PI/2);
path.PoseWpt(1, 1.0, 1.0, 0.0);
path.PoseWpt(2, 2.0, 0.0, PI/2);
path.WptZeroVelocity(0);
path.WptZeroVelocity(2);
path.ControlIntervalCounts({40, 40});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```

### Example 4: Swerve, ending velocity

```cpp
SwervePathBuilder path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWpt(0, 0.0, 0.0, 0.0);
path.PoseWpt(1, 0.0, 1.0, 0.0);
path.WptZeroVelocity(0);
path.WptVelocityPolar(1, 0.2, 0.0);
path.ControlIntervalCounts({40});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```

### Example 5: Swerve, circle obstacle

```cpp
SwervePathBuilderBuilder path;
path.SetDrivetrain(swerveDrivetrain);
path.PoseWpt(0, 0.0 0_m, 0_deg);
Obstacle obstacle{
    .safetyDistance = 0.1, // radius of 0.1
    .points = {{0.5, 0.5}}
};
path.SgmtObstacle(0, 1, obstacle);
path.PoseWpt(1, 1.0, 0.0, 0.0);
path.WptZeroVelocity(0);
path.WptZeroVelocity(1);
path.ControlIntervalCounts({40});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```

### Example 6: Approach a pick up station at a certain direction

```cpp
SwervePathBuilder path;
path.SetDrivetrain(swerveDrivetrain);

// starting position
path.PoseWpt(0, 0.0, 0.0, 0.0);

// align towards the station one meter behind
path.PoseWpt(1, 1.0, 1.0, PI/2);
path.WptZeroAngularVelocity(1);
path.WptVelocityDirection(1, PI/2);

// go up to the station
// In practice, the optimizer will still end up aligning the heading without
// the pose constraint since it's most optimal
path.TranslationWpt(2, 1.0, 2.0);

// realign behind the station
path.PoseWpt(3, 1.0, 1.0, PI/2);
path.WptZeroAngularVelocity(3);
path.WptVelocityDirection(3, PI/2);

// ending position
path.PoseWpt(4, 2.0, 0.0, PI);

path.WptZeroVelocity(0);
path.WptZeroVelocity(4);
path.ControlIntervalCounts({40, 30, 30, 40});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```

### Example 7: Circular path with a point-point constraint

Note that forcing a circular path is not a common problem in FRC, this example
is only to demonstrate how various constraints work.

```cpp
SwervePathBuilder path;
path.SetDrivetrain(swerveDrivetrain);

path.PoseWpt(0, 0.0, 0.0, 0.0);
path.SgmtConstraint(0, 1
    PointPointConstraint{
        0.0, 0.0, // robot point--center of bot
        1.0, 0.0, // field point to orbit around
        1.0});     // stay a distance of 1_m away to force circular motion

// tell optimizer to go in +y direction rather than -y
path.AddInitialGuess(0, 0.0, 0.0, 0.0);

path.PoseWpt(1, 2.0, 0.0, 0.0);

path.WptZeroVelocity(0);
path.WptZeroVelocity(1);
path.ControlIntervalCounts({30});
auto solution = OptimalTrajectoryGenerator::Generate(path);
```
