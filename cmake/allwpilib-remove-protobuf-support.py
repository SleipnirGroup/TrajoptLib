#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys

shutil.rmtree("wpimath/src/main/native/cpp/controller/proto", ignore_errors=True)
shutil.rmtree("wpimath/src/main/native/cpp/geometry/proto", ignore_errors=True)
shutil.rmtree("wpimath/src/main/native/cpp/kinematics/proto", ignore_errors=True)
shutil.rmtree("wpimath/src/main/native/cpp/system/plant/proto", ignore_errors=True)
shutil.rmtree("wpimath/src/main/native/cpp/trajectory/proto", ignore_errors=True)
shutil.rmtree("wpiutil/src/main/native/cpp/protobuf", ignore_errors=True)
shutil.rmtree(
    "wpimath/src/main/native/include/frc/controller/proto", ignore_errors=True
)
shutil.rmtree("wpimath/src/main/native/include/frc/geometry/proto", ignore_errors=True)
shutil.rmtree(
    "wpimath/src/main/native/include/frc/kinematics/proto", ignore_errors=True
)
shutil.rmtree(
    "wpimath/src/main/native/include/frc/system/plant/proto", ignore_errors=True
)
shutil.rmtree(
    "wpimath/src/main/native/include/frc/trajectory/proto", ignore_errors=True
)
shutil.rmtree("wpiutil/src/main/native/thirdparty/protobuf", ignore_errors=True)
subprocess.run(
    [
        "git",
        "apply",
        "--ignore-whitespace",
        os.path.join(sys.argv[1], "cmake/allwpilib-remove-protobuf-support.patch"),
    ],
    check=True,
)
