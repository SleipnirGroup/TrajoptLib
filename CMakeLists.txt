cmake_minimum_required(VERSION 3.21)

project(helixtrajectory LANGUAGES CXX)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

set(CASADI_DIR ${CMAKE_BINARY_DIR}/external/src/casadi-3.5.5/casadi)

set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
message (STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
if(${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
    message(STATUS "Building for mingw")
    set(CMAKE_CXX_FLAGS "-static-libgcc -static-libstdc++")
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-windows-py39-v3.5.5-64bit.zip)
elseif(APPLE)
    message(STATUS "Building for apple")
    set(CMAKE_INSTALL_RPATH "@loader_path/../lib;@loader_path")
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-osx-py39-v3.5.5.tar.gz)
elseif(UNIX)
    message(STATUS "Building for unix")
    set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN")
    add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-linux-py39-v3.5.5-64bit.tar.gz)
endif()

message(STATUS "Downloading CasADi from ${CASADI_URL}")

include(ExternalProject)
ExternalProject_Add(
    casadi-3.5.5
    URL ${CASADI_URL}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    PREFIX ${CMAKE_BINARY_DIR}/external)

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

# add_library(${PROJECT_NAME} SHARED CasADiOpti.cpp Drivetrain.cpp HolonomicAccelerationConstraint.cpp HolonomicDrivetrain.cpp HolonomicPath.cpp HolonomicTrajectoryOptimizationProblem.cpp HolonomicVelocityConstraint.cpp HolonomicWaypoint.cpp InitialGuessPoint.cpp InvalidPathException.cpp Obstacle.cpp ObstaclePoint.cpp OptimalTrajectoryGenerator.cpp Path.cpp PlanarBound.cpp PositionConstraint.cpp ScalarBound.cpp SwerveDrivetrain.cpp SwerveModule.cpp SwerveTrajectoryOptimizationProblem.cpp TrajectoryGenerationException.cpp TrajectoryOptimizationProblem.cpp TrajectorySample.cpp Waypoint.cpp JNI.cpp)

file(GLOB_RECURSE src helixtrajectorycpp/src/cpp/*.cpp)
add_executable(${PROJECT_NAME} ${src})

target_include_directories(${PROJECT_NAME} PRIVATE ${CASADI_DIR}/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/jni)
if(${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/jni/win32)
    # set(JAR_LIB_FILE_PATH "windows/x86_64/")
    set(LIB_FILENAME "lib${PROJECT_NAME}.dll")
    install(FILES ${CASADI_DIR}/libcasadi.dll   ${CASADI_DIR}/libstdc++-6.dll ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dll   ${CASADI_DIR}/libgfortran-3.dll             ${CASADI_DIR}/libquadmath-0.dll   ${CASADI_DIR}/libgcc_s_seh-1.dll DESTINATION "bin")
    # install(FILES ${CASADI_DIR}/libcasadi.dll   ${CASADI_DIR}/libstdc++-6.dll ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dll   ${CASADI_DIR}/libgfortran-3.dll             ${CASADI_DIR}/libquadmath-0.dll   ${CASADI_DIR}/libgcc_s_seh-1.dll DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
elseif(APPLE)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/jni/darwin)
    # set(JAR_LIB_FILE_PATH "osx/x86_64/")
    set(LIB_FILENAME "lib${PROJECT_NAME}.dylib")
    install(FILES ${CASADI_DIR}/libcasadi.dylib ${CASADI_DIR}/libc++.1.dylib  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dylib ${CASADI_DIR}/libgfortran.4.dylib           ${CASADI_DIR}/libquadmath.0.dylib ${CASADI_DIR}/libgcc_s.1.dylib   DESTINATION "lib")
    # install(FILES ${CASADI_DIR}/libcasadi.dylib ${CASADI_DIR}/libc++.1.dylib  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.dylib ${CASADI_DIR}/libgfortran.4.dylib           ${CASADI_DIR}/libquadmath.0.dylib ${CASADI_DIR}/libgcc_s.1.dylib   DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
elseif(UNIX)
    target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/jni/linux)
    # set(JAR_LIB_FILE_PATH "linux/x86_64/")
    set(LIB_FILENAME "lib${PROJECT_NAME}.so")
    install(FILES ${CASADI_DIR}/libcasadi.so                                  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.so    ${CASADI_DIR}/libgfortran-ed201abd.so.3.0.0                                                                    DESTINATION "lib")
    # install(FILES ${CASADI_DIR}/libcasadi.so                                  ${CASADI_DIR}/libcasadi_nlpsol_ipopt.so    ${CASADI_DIR}/libgfortran-ed201abd.so.3.0.0                                                                    DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
endif()

target_link_directories(${PROJECT_NAME} PRIVATE ${CASADI_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE casadi fmt)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/src/include)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/src/cpp)

# if(${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
#     install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION "lib")
#     install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
# else()
install(TARGETS ${PROJECT_NAME} DESTINATION "")
    # install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH})
# endif()
# install(CODE "file(MD5 ${CMAKE_BINARY_DIR}/install/lib/${LIB_FILENAME} HT_HASH)\nmessage(STATUS \"HT Hash: \${HT_HASH}\")\nfile(WRITE ${CMAKE_INSTALL_DIR}/lib/src/main/resources/${JAR_LIB_FILE_PATH}/helixtrajectory.hash \${HT_HASH})\nfile(RENAME ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH}/${LIB_FILENAME} ${CMAKE_SOURCE_DIR}/helixtrajectoryj/src/main/resources/${JAR_LIB_FILE_PATH}/${LIB_FILENAME}.\${HT_HASH})")