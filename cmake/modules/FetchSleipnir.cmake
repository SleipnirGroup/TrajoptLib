macro(fetch_sleipnir)
    include(FetchContent)

    fetchcontent_declare(
        Sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        # main on 2024-02-16
        GIT_TAG 71994a021395711cc3992603d1a226e2118a3a0a
    )

    fetchcontent_getproperties(Sleipnir)
    if(NOT Sleipnir_POPULATED)
        fetchcontent_populate(Sleipnir)
        add_subdirectory(
            ${sleipnir_SOURCE_DIR}
            ${sleipnir_BINARY_DIR}
            EXCLUDE_FROM_ALL
        )
    endif()
endmacro()
