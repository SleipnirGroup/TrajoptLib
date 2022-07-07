add_library(${PROJECT_NAME} SHARED Drive.cpp HolonomicDrive.cpp JNI.cpp Obstacle.cpp Path.cpp SwerveDrive.cpp Trajectory.cpp TrajectoryGenerator.cpp TrajectoryUtil.cpp)
# add_executable(${PROJECT_NAME} Main.cpp)
install(TARGETS ${PROJECT_NAME} DESTINATION "lib")

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include/darwin)
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(${PROJECT_NAME} PUBLIC casadi)

set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)