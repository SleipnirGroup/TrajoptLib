use cmake::Config;

fn main() {
    let mut cmake_config = Config::new(".");

    cmake_config
        .profile("Release")
        .define("BUILD_TESTING", "OFF");

    if cfg!(feature = "sleipnir") && cfg!(feature = "casadi") {
        panic!("Only select one optimizer backend via cargo `--features sleipnir` or `--features casadi`.");
    }

    if cfg!(feature = "sleipnir") {
        cmake_config
            .define("OPTIMIZER_BACKEND", "sleipnir")
            .define("BUILD_SHARED_LIBS", "OFF");

        if cfg!(target_os = "windows") {
            cmake_config
                .generator("Visual Studio 17 2022")
                .define("CMAKE_GENERATOR_PLATFORM", "x64")
                .cxxflag("/EHsc");
        } else if cfg!(target_os = "linux") {
            cmake_config
                .define("CMAKE_CXX_COMPILER", "g++")
                .define("CMAKE_C_COMPILER", "gcc");
        }
    } else if cfg!(feature = "casadi") {
        cmake_config.define("OPTIMIZER_BACKEND", "casadi");

        if cfg!(target_os = "windows") {
            cmake_config
                .generator("MinGW Makefiles")
                .define("CMAKE_CXX_COMPILER", "x86_64-w64-mingw32-g++")
                .define("CMAKE_C_COMPILER", "x86_64-w64-mingw32-gcc")
                .define(
                    "CMAKE_SHARED_LINKER_FLAGS",
                    "-static-libgcc -static-libstdc++",
                )
                .define("CMAKE_EXE_LINKER_FLAGS", "-static-libgcc -static-libstdc++");
        } else if cfg!(target_os = "linux") {
            cmake_config
                .define("CMAKE_CXX_COMPILER", "g++")
                .define("CMAKE_C_COMPILER", "gcc");
        }
    } else {
        panic!(
            "Select an optimizer backend via cargo `--features sleipnir` or `--features casadi`."
        );
    }

    let cmake_dest = cmake_config.build();

    let mut bridge_build = cxx_build::bridge("src/lib.rs");

    bridge_build
        .file("src/trajoptlibrust.cpp")
        .include("src")
        .include(format!("{}/include", cmake_dest.display()))
        .std("c++20");

    if cfg!(feature = "casadi") && cfg!(target_os = "linux") {
        bridge_build.define("_GLIBCXX_USE_CXX11_ABI", "0");
    }

    bridge_build.compile("trajoptrust");

    println!(
        "cargo:rustc-link-search=native={}/bin",
        cmake_dest.display()
    );
    println!(
        "cargo:rustc-link-search=native={}/lib",
        cmake_dest.display()
    );
    println!("cargo:rustc-link-lib=trajoptrust");
    println!("cargo:rustc-link-lib=TrajoptLib");
    if cfg!(feature = "sleipnir") {
        println!("cargo:rustc-link-lib=Sleipnir");
        println!("cargo:rustc-link-lib=fmt");
    }

    println!("cargo:rerun-if-changed=src/trajoptlibrust.h");
    println!("cargo:rerun-if-changed=src/trajoptlibrust.cpp");
    println!("cargo:rerun-if-changed=src/lib.rs");
}
