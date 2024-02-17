use cmake::Config;

fn main() -> miette::Result<()> {
    let mut cmake_config = Config::new("..");
    let optimizer_backend = "casadi";

    cmake_config
        .profile("RelWithDebInfo")
        .define("OPTIMIZER_BACKEND", optimizer_backend)
        .define("BUILD_TESTING", "OFF");

    if cfg!(target_os = "windows") {
        if optimizer_backend == "casadi" {
            cmake_config
                .generator("MinGW Makefiles")
                .define("CMAKE_CXX_COMPILER", "x86_64-w64-mingw32-g++")
                .define("CMAKE_C_COMPILER", "x86_64-w64-mingw32-gcc")
                .define(
                    "CMAKE_SHARED_LINKER_FLAGS",
                    "-static-libgcc -static-libstdc++",
                )
                .define("CMAKE_EXE_LINKER_FLAGS", "-static-libgcc -static-libstdc++");
        } else if optimizer_backend == "sleipnir" {
            cmake_config
                .generator("Visual Studio 17 2022")
                .define("CMAKE_GENERATOR_PLATFORM", "x64")
                .cxxflag("/EHsc");
        }
    }

    if cfg!(target_os = "linux") {
        cmake_config
            .define("CMAKE_CXX_COMPILER", "g++")
            .define("CMAKE_C_COMPILER", "gcc");
    }

    if optimizer_backend == "sleipnir" {
        cmake_config.define("BUILD_SHARED_LIBS", "OFF");
    }

    let dst = cmake_config.build();

    println!("cargo:rustc-link-search=native={}/bin", dst.display());
    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=TrajoptLib");
    if optimizer_backend == "sleipnir" {
        println!("cargo:rustc-link-lib=Sleipnir");
        println!("cargo:rustc-link-lib=fmt");
    }

    cxx_build::bridge("src/lib.rs") // returns a cc::Build
        .file("src/trajoptlib.cc")
        .include("include")
        .include(format!("{}/include", dst.display()))
        .flag_if_supported("/std:c++20")
        .flag_if_supported("-std=c++20")
        .compile("trajoptlib-rust");

    println!("cargo:rerun-if-changed=include/trajoptlib.h");
    println!("cargo:rerun-if-changed=src/trajoptlib.cc");
    println!("cargo:rerun-if-changed=src/lib.rs");
    Ok(())
}
