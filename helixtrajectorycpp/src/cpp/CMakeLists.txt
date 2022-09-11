add_executable(${PROJECT_NAME} CasADiOpti.cpp Drivetrain.cpp HolonomicDrivetrain.cpp HolonomicPath.cpp HolonomicTrajectory.cpp HolonomicTrajectoryOptimizationProblem.cpp HolonomicTrajectorySample.cpp HolonomicTrajectorySegment.cpp HolonomicWaypoint.cpp InitialGuessPoint.cpp InvalidPathException.cpp Obstacle.cpp ObstaclePoint.cpp OptimalTrajectoryGenerator.cpp Path.cpp SwerveDrivetrain.cpp SwerveModule.cpp SwerveTrajectoryOptimizationProblem.cpp TrajectoryGenerationException.cpp TrajectoryOptimizationProblem.cpp TrajectorySample.cpp Waypoint.cpp Main.cpp)
# add_library(${PROJECT_NAME} SHARED CasADiHolonomicTrajectoryOptimizationProblem.cpp CasADiSwerveTrajectoryOptimizationProblem.cpp CasADiTrajectoryOptimizationProblem.cpp Drivetrain.cpp HolonomicDrivetrain.cpp HolonomicPath.cpp HolonomicTrajectory.cpp HolonomicTrajectorySample.cpp HolonomicTrajectorySegment.cpp HolonomicWaypoint.cpp InitialGuessPoint.cpp InvalidPathException.cpp Obstacle.cpp ObstaclePoint.cpp OptimalTrajectoryGenerator.cpp Path.cpp SwerveDrivetrain.cpp SwerveModule.cpp TrajectoryGenerationException.cpp TrajectorySample.cpp Waypoint.cpp JNI.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/lib/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include/darwin)

target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE casadi)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/helixtrajectorycpp/src/include)

install(TARGETS ${PROJECT_NAME} DESTINATION "lib")