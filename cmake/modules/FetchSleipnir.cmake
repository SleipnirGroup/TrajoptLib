macro(fetch_sleipnir)

    include(FetchContent)
    FetchContent_Declare(
        sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        GIT_TAG b43f7ed86d9925de5da33dbca751cef315a12b62
    )
    FetchContent_MakeAvailable(sleipnir)
endmacro()