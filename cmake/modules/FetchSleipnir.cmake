macro(fetch_sleipnir)
    include(FetchContent)

    fetchcontent_declare(
        Sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        # main on 2024-01-07
        GIT_TAG da2bd2bec834ab4154eb5b0010818e7edda37406
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
