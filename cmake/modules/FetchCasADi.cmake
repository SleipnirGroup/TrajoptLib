macro(fetch_casadi)
  if (${CMAKE_SYSTEM_NAME} MATCHES "MINGW" OR ${CMAKE_SYSTEM_NAME} MATCHES "MSYS" OR WIN32)
    message(STATUS "Building for Windows")
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-windows-py39-v3.5.5-64bit.zip)
  elseif (APPLE)
    message(STATUS "Building for macOS")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};@loader_path/../lib;@loader_path")
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/casadi-osx-py39-v3.5.5.tar.gz)
  elseif (UNIX)
    message(STATUS "Building for Linux")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};$ORIGIN/../lib;$ORIGIN")
    set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.5.5/libcasadi-linux-gcc5-v3.5.5.tar.gz)
  endif()
  message(STATUS "Downloading CasADi from ${CASADI_URL}")

  include(FetchContent)

  FetchContent_Declare(
    casadi
    URL ${CASADI_URL}
  )

  FetchContent_MakeAvailable(casadi)
endmacro()
