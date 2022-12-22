macro(fetch_sleipnir)

    include(FetchContent)
    FetchContent_Declare(
        sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        GIT_TAG 3cf29ed4559815b1e7b7154fff7f5b7f2b72a0f8
    )
    FetchContent_MakeAvailable(sleipnir)
endmacro()