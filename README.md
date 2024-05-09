# TrajoptLib

![Build](https://github.com/SleipnirGroup/TrajoptLib/actions/workflows/build.yml/badge.svg)
[![C++ Documentation](https://img.shields.io/badge/documentation-c%2B%2B-blue)](https://sleipnirgroup.github.io/TrajoptLib/)
[![Discord](https://img.shields.io/discord/975739302933856277?color=%23738ADB&label=Join%20our%20Discord&logo=discord&logoColor=white)](https://discord.gg/ad2EEZZwsS)

This library is used to generate time-optimal trajectories for FRC robots.

## Trajectory Optimization

Trajectory optimization works by mathematically formulating the problem of travelling along a given path with the minimum possible time. The physical constraints of motor power capacity are applied along with waypoint constraints, which force the robot to begin and end a segment of the trajectory with a certain state. A mathematical solver must vary the position of the robot at each discrete timestamp to minimize total time.

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

## Dependencies

* C++20 compiler
  * On Windows, install [Visual Studio Community 2022](https://visualstudio.microsoft.com/vs/community/) and select the C++ programming language during installation
  * On Linux, install GCC 11 or greater via `sudo apt install gcc`
  * On macOS, install the Xcode command-line build tools via `xcode-select --install`. Xcode 15.0.1 or later is required.
* [CMake](https://cmake.org/download/) 3.21 or greater
  * On Windows, install from the link above
  * On Linux, install via `sudo apt install cmake`
  * On macOS, install via `brew install cmake`
* [Rust](https://www.rust-lang.org/) compiler
* [Sleipnir](https://github.com/SleipnirGroup/Sleipnir) (optional backend)
* [CasADi](https://github.com/casadi/casadi) (optional backend)
* [Catch2](https://github.com/catchorg/Catch2) (tests only)

Library dependencies which aren't installed locally will be automatically downloaded and built by CMake.

## Build instructions

On Windows, open a [Developer PowerShell](https://learn.microsoft.com/en-us/visualstudio/ide/reference/command-prompt-powershell?view=vs-2022). On Linux or macOS, open a Bash shell.

Clone the repository.
```bash
git clone git@github.com:SleipnirGroup/TrajoptLib
cd TrajoptLib
```

### C++ library

```bash
# Configure with Sleipnir backend; automatically downloads library dependencies
cmake -B build -S . -DOPTIMIZER_BACKEND=sleipnir

# Configure with CasADi backend; automatically downloads library dependencies
cmake -B build -S . -DOPTIMIZER_BACKEND=casadi

# Build
cmake --build build

# Test
ctest --test-dir build --output-on-failure

# Install
cmake --install build --prefix pkgdir
```

The following build types can be specified via `-DCMAKE_BUILD_TYPE` during CMake configure:

* Debug
  * Optimizations off
  * Debug symbols on
* Release
  * Optimizations on
  * Debug symbols off
* RelWithDebInfo (default)
  * Release build type, but with debug info
* MinSizeRel
  * Minimum size release build

### Rust library

```bash
cargo build --features sleipnir  # Sleipnir backend
cargo build --features casadi  # CasADi backend
```
