# TrajoptLib

![Build](https://github.com/SleipnirGroup/TrajoptLib/actions/workflows/build.yml/badge.svg)
[![C++ Documentation](https://img.shields.io/badge/documentation-c%2B%2B-blue)](https://sleipnirgroup.github.io/TrajoptLib/)
[![Discord](https://img.shields.io/discord/975739302933856277?color=%23738ADB&label=Join%20our%20Discord&logo=discord&logoColor=white)](https://discord.gg/ad2EEZZwsS)

This library is used to generate time-optimal trajectories for FRC robots.

## Trajectory Optimization
Trajectory optimization works by mathematically formulating the problem of
travelling along a given path with the minimum possible time. The physical
constraints of motor power capacity are applied along with waypoint constraints,
which force the robot to begin and end a segment of the trajectory with a certain
state. A mathematical solver must vary the position of the robot at each discrete
timestamp to minimize total time.

## Features
### Trajopt
* Currently only supports swerve drives with arbitray module configurations
* Position and velocity constraints at each waypoint
* Circle and polygon obstacle avoidance
* Custom physical constraints of robot
* Custom bumper shape

### API
* Java and C++ API with almost complete parity
* CasADi dependencies bundled with Jar
