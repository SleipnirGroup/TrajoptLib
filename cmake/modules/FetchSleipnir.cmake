macro(fetch_sleipnir)
    include(FetchContent)

    fetchcontent_declare(
        Sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        # main on 2024-02-15
        GIT_TAG b20cf8f393143e0c7aef08f9d7e728eb6afb6ad1
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
