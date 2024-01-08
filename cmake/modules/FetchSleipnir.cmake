macro(fetch_sleipnir)
    include(FetchContent)

    fetchcontent_declare(
        Sleipnir
        GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
        # main on 2024-01-08
        GIT_TAG 02c5b2446975b45b38f35f4a40e54f988010d486
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
