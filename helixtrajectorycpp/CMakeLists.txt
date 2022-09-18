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

add_subdirectory(src)