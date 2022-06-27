cmake_minimum_required(VERSION 3.10.2)

project(helixtrajectorycpp LANGUAGES CXX)
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(src)