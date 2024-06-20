#!/usr/bin/env python3

import os
import shutil
import subprocess
import sys


def make_emscripten_backtrace_a_no_op():
    subprocess.run(
        [
            "git",
            "apply",
            "--ignore-whitespace",
            os.path.join(
                sys.argv[1], "cmake/allwpilib-make-emscripten-backtrace-a-no-op.patch"
            ),
        ],
        check=True,
    )


def remove_protobuf_support():
    shutil.rmtree("wpimath/src/main/native/cpp/controller/proto", ignore_errors=True)
    shutil.rmtree("wpimath/src/main/native/cpp/geometry/proto", ignore_errors=True)
    shutil.rmtree("wpimath/src/main/native/cpp/kinematics/proto", ignore_errors=True)
    shutil.rmtree("wpimath/src/main/native/cpp/system/plant/proto", ignore_errors=True)
    shutil.rmtree("wpimath/src/main/native/cpp/trajectory/proto", ignore_errors=True)
    shutil.rmtree("wpiutil/src/main/native/cpp/protobuf", ignore_errors=True)
    shutil.rmtree(
        "wpimath/src/main/native/include/frc/controller/proto", ignore_errors=True
    )
    shutil.rmtree(
        "wpimath/src/main/native/include/frc/geometry/proto", ignore_errors=True
    )
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


def disable_psabi_warning():
    subprocess.run(
        [
            "git",
            "apply",
            "--ignore-whitespace",
            os.path.join(sys.argv[1], "cmake/allwpilib-disable-psabi-warning.patch"),
        ],
        check=True,
    )


def main():
    make_emscripten_backtrace_a_no_op()
    remove_protobuf_support()
    disable_psabi_warning()


if __name__ == "__main__":
    main()
