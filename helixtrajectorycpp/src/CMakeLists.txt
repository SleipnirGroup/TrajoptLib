target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include)
target_include_directories(${PROJECT_NAME} PRIVATE /Library/Java/JavaVirtualMachines/jdk-16.0.1.jdk/Contents/Home/include/darwin)

target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/lib)
target_link_libraries(${PROJECT_NAME} PRIVATE casadi)

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/include)

# set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

add_subdirectory(cpp)