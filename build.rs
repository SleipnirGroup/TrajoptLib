use cmake::Config;

fn main() {
    let mut cmake_config = Config::new(".");

    cmake_config
        .profile("Release")
        .define("BUILD_TESTING", "OFF");

    if cfg!(feature = "sleipnir") {
        cmake_config
            .define("OPTIMIZER_BACKEND", "sleipnir")
            .define("BUILD_SHARED_LIBS", "OFF");

        if cfg!(target_os = "windows") {
            cmake_config
                .generator("Visual Studio 17 2022")
                .cxxflag("/EHsc");
        }
    } else {
        panic!("Select an optimizer backend via cargo `--features sleipnir`.");
    }

    let cmake_dest = cmake_config.build();

    let mut bridge_build = cxx_build::bridge("src/lib.rs");

    bridge_build
        .file("src/trajoptlibrust.cpp")
        .include("src")
        .include(format!("{}/include", cmake_dest.display()))
        .std("c++20");

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
