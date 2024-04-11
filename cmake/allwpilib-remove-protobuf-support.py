#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys

shutil.rmtree("wpimath/src/main/native/cpp/controller/proto")
shutil.rmtree("wpimath/src/main/native/cpp/geometry/proto")
shutil.rmtree("wpimath/src/main/native/cpp/kinematics/proto")
shutil.rmtree("wpimath/src/main/native/cpp/system/plant/proto")
shutil.rmtree("wpimath/src/main/native/cpp/trajectory/proto")
shutil.rmtree("wpiutil/src/main/native/cpp/protobuf")
shutil.rmtree("wpiutil/src/main/native/thirdparty/protobuf")
subprocess.run(
    [
        "git",
        "apply",
        os.path.join(sys.argv[1], "cmake/allwpilib-remove-protobuf-support.patch"),
    ]
)
