# Disable in-source builds to prevent source tree corruption
if ("${CMAKE_CURRENT_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

cmake_minimum_required(VERSION 3.21)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

project(helixtrajectory LANGUAGES CXX)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

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
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
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

# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS FALSE)

include(FetchCasADi)
fetch_casadi()

include(FetchSleipnir)
fetch_sleipnir()

set(CMAKE_CXX_STANDARD 20)

file(GLOB_RECURSE src helixtrajectorycpp/src/cpp/*.cpp)
add_executable(helixtrajectory ${src})

target_include_directories(helixtrajectory PRIVATE ${CASADI_DIR}/include)
target_include_directories(helixtrajectory PRIVATE ${CMAKE_SOURCE_DIR}/jni)
if(${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
    target_include_directories(helixtrajectory PRIVATE ${CMAKE_SOURCE_DIR}/jni/win32)
    # set(JAR_LIB_FILE_PATH "windows/x86_64/")
    set(LIB_FILENAME "libhelixtrajectory.dll")
    install(FILES ${CASADI_DIR}/libcasadi.dll   ${CASADI_DIR}/libstdc++-6.dll ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dll   ${CASADI_DIR}/libgfortran-3.dll             ${CASADI_DIR}/libquadmath-0.dll   ${CASADI_DIR}/libgcc_s_seh-1.dll DESTINATION "bin")
    # install(FILES ${CASADI_DIR}/libcasadi.dll   ${CASADI_DIR}/libstdc++-6.dll ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dll   ${CASADI_DIR}/libgfortran-3.dll             ${CASADI_DIR}/libquadmath-0.dll   ${CASADI_DIR}/libgcc_s_seh-1.dll DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
elseif(APPLE)
    target_include_directories(helixtrajectory PRIVATE ${CMAKE_SOURCE_DIR}/jni/darwin)
    # set(JAR_LIB_FILE_PATH "osx/x86_64/")
    set(LIB_FILENAME "libhelixtrajectory.dylib")
    install(FILES ${CASADI_DIR}/libcasadi.dylib ${CASADI_DIR}/libc++.1.dylib  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dylib ${CASADI_DIR}/libgfortran.4.dylib           ${CASADI_DIR}/libquadmath.0.dylib ${CASADI_DIR}/libgcc_s.1.dylib   DESTINATION "lib")
    # install(FILES ${CASADI_DIR}/libcasadi.dylib ${CASADI_DIR}/libc++.1.dylib  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dylib ${CASADI_DIR}/libgfortran.4.dylib           ${CASADI_DIR}/libquadmath.0.dylib ${CASADI_DIR}/libgcc_s.1.dylib   DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
elseif(UNIX)
    target_include_directories(helixtrajectory PRIVATE ${CMAKE_SOURCE_DIR}/jni/linux)
    # set(JAR_LIB_FILE_PATH "linux/x86_64/")
    set(LIB_FILENAME "libhelixtrajectory.so")
    install(FILES ${CASADI_DIR}/libcasadi.so                                  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.so    ${CASADI_DIR}/libgfortran-ed201abd.so.3.0.0                                                                    DESTINATION "lib")
    # install(FILES ${CASADI_DIR}/libcasadi.so                                  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.so    ${CASADI_DIR}/libgfortran-ed201abd.so.3.0.0                                                                    DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
endif()

target_link_directories(helixtrajectory PRIVATE ${CASADI_DIR})
target_link_libraries(helixtrajectory PRIVATE casadi fmt Sleipnir)
target_include_directories(helixtrajectory PUBLIC ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/src/include)
target_include_directories(helixtrajectory PRIVATE ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/src/cpp)

# if(${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
#     install(TARGETS helixtrajectory RUNTIME DESTINATION "lib")
#     install(TARGETS helixtrajectory RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
# else()


include(FetchContent)
set(BUILD_SHARED_LIBS_SAVE ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
FetchContent_Declare(
  fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        9.1.0
)
FetchContent_MakeAvailable(fmt)
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVE})

install(DIRECTORY "helixtrajectorycpp/src/include" DESTINATION "./")

install(TARGETS helixtrajectory DESTINATION "")
    # install(TARGETS helixtrajectory DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
# endif()
# install(CODE "file(MD5 ${CMAKE_BINARY_DIR}/install/lib/${LIB_FILENAME} HT_HASH)\nmessage(STATUS \"HT Hash: \${HT_HASH}\")\nfile(WRITE ${CMAKE_INSTALL_DIR}/lib/src/main/resources/${JAR_LIB_FILE_PATH}/helixtrajectory.hash \${HT_HASH})\nfile(RENAME ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH}/${LIB_FILENAME} ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH}/${LIB_FILENAME}.\${HT_HASH})")