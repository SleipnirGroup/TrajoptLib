macro(fetch_sleipnir)

    set(BUILD_SHARED_LIBS_SAVE ${BUILD_SHARED_LIBS})
    set(BUILD_SHARED_LIBS OFF)
    set(INSTALL_SLEIPNIR OFF)
    # set(BUILD_TESTING OFF)
    include(FetchContent)
    FetchContent_Declare(
        sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        GIT_TAG c8d2d861f19b8975be71f324433511550317cf0a
        CMAKE_ARGS -DBUILD_TESTING=OFF -DBUILD_BENCHMARKING=OFF -DINSTALL_GMOCK=OFF -DINSTALL_GTEST=OFF
    )
    FetchContent_MakeAvailable(sleipnir)
    set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_SAVE})

    set(OPTIMIZER_LIBS Sleipnir)
endmacro()