#!/bin/bash
rm -r wpimath/src/main/native/cpp/controller/proto
rm -r wpimath/src/main/native/cpp/geometry/proto
rm -r wpimath/src/main/native/cpp/kinematics/proto
rm -r wpimath/src/main/native/cpp/system/plant/proto
rm -r wpimath/src/main/native/cpp/trajectory/proto
rm -r wpiutil/src/main/native/cpp/protobuf
rm -r wpiutil/src/main/native/thirdparty/protobuf
git apply $1/cmake/allwpilib-remove-protobuf-support.patch
