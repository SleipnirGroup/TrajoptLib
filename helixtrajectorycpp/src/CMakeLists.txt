add_executable(${PROJECT_NAME} Drivetrain.cpp HolonomicDrivetrain.cpp HolonomicPath.cpp HolonomicTrajectory.cpp HolonomicTrajectorySample.cpp HolonomicTrajectorySegment.cpp HolonomicWaypoint.cpp InitialGuessPoint.cpp Obstacle.cpp ObstaclePoint.cpp OptimalHolonomicTrajectoryGenerator.cpp OptimalTrajectoryGenerator.cpp Path.cpp SwerveDrivetrain.cpp SwerveModule.cpp TrajectorySample.cpp TrajectorySegment.cpp TrajectoryUtil.cpp Waypoint.cpp Main.cpp)
# add_executable(${PROJECT_NAME} Main.cpp)
install(TARGETS ${PROJECT_NAME} DESTINATION "lib")

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include/darwin)
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(${PROJECT_NAME} PUBLIC casadi)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)