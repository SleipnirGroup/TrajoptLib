# Disable in-source builds to prevent source tree corruption
if ("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

cmake_minimum_required(VERSION 3.21)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules")

project(TrajoptLib LANGUAGES CXX)

# Use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# When building, don't use the install RPATH already (but later on when
# installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if ("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

# Set default build type to release with debug info (i.e. release mode
# optimizations are performed, but debug info still exists).
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "" FORCE)
endif()

# Generate compile_commands.json by default
if (NOT CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS "YES" CACHE STRING "" FORCE)
endif()

# Control where the static and shared libraries are built so that on Windows,
# we don't need to tinker with the path to run the executable
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS FALSE)

include(TrajoptLibCompilerFlags)

file(GLOB_RECURSE src src/*.cpp)
add_library(TrajoptLib ${src})
trajoptlib_compiler_flags(TrajoptLib)
target_include_directories(TrajoptLib
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

set_target_properties(TrajoptLib PROPERTIES DEBUG_POSTFIX "d")

set_property(TARGET TrajoptLib PROPERTY FOLDER "libraries")
target_compile_definitions(TrajoptLib PRIVATE TRAJOPT_EXPORTS)

include(FetchContent)

# fmt dependency
set(FMT_INSTALL ON)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        9.1.0
)
FetchContent_MakeAvailable(fmt)

target_link_libraries(TrajoptLib PRIVATE fmt)

set(OPTIMIZER_BACKEND "casadi" CACHE STRING "Optimizer backend")
set_property(CACHE OPTIMIZER_BACKEND PROPERTY STRINGS casadi sleipnir)

if (${OPTIMIZER_BACKEND} STREQUAL "casadi")
  message(STATUS "Using CasADi optimizer")
  include(FetchCasADi)
  fetch_casadi()
  target_compile_definitions(TrajoptLib PRIVATE OPTIMIZER_BACKEND_CASADI)
  if (${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
    target_include_directories(TrajoptLib SYSTEM PRIVATE ${CMAKE_BINARY_DIR}/_deps/casadi-src/casadi/include)
    target_link_directories(TrajoptLib PRIVATE ${CMAKE_BINARY_DIR}/_deps/casadi-src/casadi)
  elseif (APPLE)
    target_include_directories(TrajoptLib SYSTEM PRIVATE ${CMAKE_BINARY_DIR}/_deps/casadi-src/casadi/include)
    target_link_directories(TrajoptLib PRIVATE ${CMAKE_BINARY_DIR}/_deps/casadi-src/casadi)
  elseif (UNIX)
    target_include_directories(TrajoptLib SYSTEM PRIVATE ${CMAKE_BINARY_DIR}/_deps/casadi-src/include)
    target_link_directories(TrajoptLib PRIVATE ${CMAKE_BINARY_DIR}/_deps/casadi-src/lib)
  endif()
  target_link_libraries(TrajoptLib PRIVATE casadi)

  # Install CasADi libraries since FetchContent setting that up properly
  if (${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
    target_include_directories(TrajoptLib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/win32)
    set(CASADI_LIBDIR ${CMAKE_BINARY_DIR}/_deps/casadi-src/casadi)
    install(FILES
      ${CASADI_LIBDIR}/libcasadi.dll
      ${CASADI_LIBDIR}/libstdc++-6.dll
      ${CASADI_LIBDIR}/libcasadi_nlpsol_ipopt.dll
      ${CASADI_LIBDIR}/libquadmath-0.dll
      ${CASADI_LIBDIR}/libgcc_s_seh-1.dll
      DESTINATION "bin")
  elseif (APPLE)
    target_include_directories(TrajoptLib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/darwin)
    set(CASADI_LIBDIR ${CMAKE_BINARY_DIR}/_deps/casadi-src/casadi)
    install(FILES
      ${CASADI_LIBDIR}/libcasadi.dylib
      ${CASADI_LIBDIR}/libc++.1.dylib
      ${CASADI_LIBDIR}/libcasadi_nlpsol_ipopt.dylib
      ${CASADI_LIBDIR}/libquadmath.0.dylib
      ${CASADI_LIBDIR}/libgcc_s.1.dylib
      DESTINATION "lib")
  elseif (UNIX)
    target_include_directories(TrajoptLib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/jni/linux)
    set(CASADI_LIBDIR ${CMAKE_BINARY_DIR}/_deps/casadi-src/lib)
    install(FILES
      ${CASADI_LIBDIR}/libcasadi.so
      ${CASADI_LIBDIR}/libcasadi_nlpsol_ipopt.so
      DESTINATION "lib")
  endif()
elseif(${OPTIMIZER_BACKEND} STREQUAL "sleipnir")
  message(STATUS "Using Sleipnir optimizer")
  include(FetchSleipnir)
  fetch_sleipnir()
  target_compile_definitions(TrajoptLib PRIVATE OPTIMIZER_BACKEND_SLEIPNIR)
  target_link_libraries(TrajoptLib PRIVATE Sleipnir)
endif()

target_include_directories(TrajoptLib
                           PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:include/TrajoptLib>)

install(TARGETS TrajoptLib
        EXPORT TrajoptLibTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)
export(TARGETS TrajoptLib
       FILE TrajoptLib.cmake
       NAMESPACE TrajoptLib::)
install(DIRECTORY include/ DESTINATION "include/TrajoptLib")
install(EXPORT TrajoptLibTargets
        FILE TrajoptLib.cmake
        NAMESPACE TrajoptLib::
        DESTINATION lib/cmake/TrajoptLib)

include(CMakePackageConfigHelpers)

# Generate the config file that includes the exports
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TrajoptLibConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/TrajoptLibConfig.cmake
  INSTALL_DESTINATION "lib/cmake/TrajoptLib"
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

# Install the config file
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/TrajoptLibConfig.cmake
  DESTINATION lib/cmake/TrajoptLib)

# Build examples and example tests
include(TrajoptLibSubdirList)
trajoptlib_subdir_list(EXAMPLES ${CMAKE_CURRENT_SOURCE_DIR}/examples)
foreach(example ${EXAMPLES})
  # Build example
  file(GLOB_RECURSE sources examples/${example}/src/*.cpp)
  add_executable(${example} ${sources})
  trajoptlib_compiler_flags(${example})
  target_include_directories(${example}
                             PRIVATE
                             ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/include
                             ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/units/include)
  target_link_libraries(${example}
                        PRIVATE
                        TrajoptLib
                        fmt)

  # Build example test if files exist for it
  if (BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/test)
    file(GLOB_RECURSE test_sources examples/${example}/test/*.cpp)
    add_executable(${example}Test ${sources} ${test_sources})
    target_include_directories(${example}Test
                               PRIVATE
                               ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/src
                               ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/test
                               ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/units/include)
    trajoptlib_compiler_flags(${example}Test)
    target_compile_definitions(${example}Test PUBLIC RUNNING_TESTS)
    target_include_directories(${example}Test
                               PRIVATE
                               ${CMAKE_CURRENT_SOURCE_DIR}/examples/${example}/include
                               ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/units/include)
    target_link_libraries(${example}Test
                          PRIVATE
                          TrajoptLib
                          fmt
                          GTest::gtest
                          GTest::gtest_main)
    if (NOT CMAKE_TOOLCHAIN_FILE)
      gtest_discover_tests(${example}Test)
    endif()
  endif()
endforeach()
