macro(fetch_casadi)
    if(${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
        message(STATUS "Building for mingw")
        set(CMAKE_CXX_FLAGS "-static-libgcc -static-libstdc++")
        set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-windows-py39-v3.5.5-64bit.zip)
    elseif(APPLE)
        message(STATUS "Building for apple")
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};@loader_path/../lib;@loader_path")
        set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-osx-py39-v3.5.5.tar.gz)
    elseif(UNIX)
        message(STATUS "Building for unix")
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};$ORIGIN/../lib;$ORIGIN")
        add_definitions(-D_GLIBCXX_USE_CXX11_ABI=0)
        set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-linux-py39-v3.5.5-64bit.tar.gz)
    endif()
    message(STATUS "Downloading CasADi from ${CASADI_URL}")
    # include(ExternalProject)
    # ExternalProject_Add(
    #     casadi-3.5.5
    #     URL ${CASADI_URL}
    #     CONFIGURE_COMMAND ""
    #     BUILD_COMMAND ""
    #     INSTALL_COMMAND ""
    #     PREFIX ${CMAKE_BINARY_DIR}/external)
    set(CASADI_DIR ${CMAKE_BINARY_DIR}/_deps/casadi-3.5.5-src/casadi)

    include(FetchContent)
    FetchContent_Declare(
        casadi-3.5.5
        URL ${CASADI_URL}
    )
    FetchContent_MakeAvailable(casadi-3.5.5)
endmacro()