cmake_minimum_required(VERSION 3.21)

project(trajoptlib-rust)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(trajoptlib-rust-cpp-interface SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/mylibrary.cpp)

target_include_directories(trajoptlib-rust-cpp-interface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(OPTIMIZER_BACKEND casadi)

set(CMAKE_SHARED_LINKER_FLAGS "-static-libgcc -static-libstdc++")
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_SOURCE_DIR}/../../build)

target_link_libraries(trajoptlib-rust-cpp-interface PRIVATE TrajoptLib)

install(TARGETS trajoptlib-rust-cpp-interface
    RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "bin"
    )



