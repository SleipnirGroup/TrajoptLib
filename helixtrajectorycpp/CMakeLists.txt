set(CMAKE_CXX_STANDARD 17)

if(APPLE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-osx-py39-v3.5.5.tar.gz)
elseif(UNIX)
    add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-linux-py39-v3.5.5-64bit.tar.gz)
elseif(WIN32)
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-windows-py39-v3.5.5-64bit.zip)
endif()

include(ExternalProject)
ExternalProject_Add(
    casadi-3.5.5
    URL ${CASADI_URL}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    PREFIX ${CMAKE_BINARY_DIR}/external)

add_subdirectory(src)