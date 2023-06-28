use cmake::Config;

fn main() -> miette::Result<()> {

  let mut cmake_config = Config::new("cpp");
  
  cmake_config.profile("RelWithDebInfo");

  if cfg!(target_os = "windows") {
    cmake_config.generator("MinGW Makefiles")
        .define("CMAKE_CXX_COMPILER", "x86_64-w64-mingw32-g++")
        .define("CMAKE_C_COMPILER", "x86_64-w64-mingw32-gcc")
        .define("CMAKE_SHARED_LINKER_FLAGS", "-static-libgcc -static-libstdc++")
        .define("CMAKE_EXE_LINKER_FLAGS", "-static-libgcc -static-libstdc++");
  }

  if cfg!(target_os = "macos") {
    if cfg!(target_arch = "aarch64") {
      cmake_config.define("CMAKE_APPLE_SILICON_PROCESSOR", "arm64");
    } else {
      cmake_config.define("CMAKE_APPLE_SILICON_PROCESSOR", "x86_64");
    }
  }
  if cfg!(target_os = "linux") {
    cmake_config
        .define("CMAKE_CXX_COMPILER", "g++")
        .define("CMAKE_C_COMPILER", "gcc");
  }

  let dst = cmake_config.build();

  println!("cargo:rustc-link-search=native={}/bin", dst.display());
  println!("cargo:rustc-link-search=native={}/lib", dst.display());
  println!("cargo:rustc-link-lib=trajoptlib-rust-cpp-interface");

  let inc_path = std::path::PathBuf::from("cpp/include");
  let mut b = autocxx_build::Builder::new("src/main.rs", &[&inc_path]).build()?;

  b.flag_if_supported("-std=c++20")
   .compile("rust-cpp-cmake-bindings");
  println!("cargo:rerun-if-changed=src/main.rs");
  println!("cargo:rerun-if-changed=cpp/CMakeLists.txt");
  Ok(())
}
