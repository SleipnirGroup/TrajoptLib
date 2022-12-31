macro(fetch_sleipnir)

    include(FetchContent)
    FetchContent_Declare(
        sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        GIT_TAG df45ae5756917b931869ea7bcd77904317471288
    )
    FetchContent_MakeAvailable(sleipnir)
endmacro()