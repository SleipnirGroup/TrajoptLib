cmake_minimum_required(VERSION 3.10)

project(helixtrajectory LANGUAGES CXX)

add_subdirectory(helixtrajectorycpp)

set(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)