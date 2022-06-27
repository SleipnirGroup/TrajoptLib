add_library(${PROJECT_NAME} SHARED Path.cpp SwerveDrive.cpp Trajectory.cpp TrajectoryGenerator.cpp TrajectoryUtil.cpp JNI.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../casadi/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/zulu-17.jdk/Contents/Home/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/zulu-17.jdk/Contents/Home/include/darwin)
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../casadi)
target_link_libraries(${PROJECT_NAME} PUBLIC casadi;casadi_nlpsol_ipopt)