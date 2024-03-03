macro(trajoptlib_compiler_flags target)
    if(NOT MSVC)
        target_compile_options(
            ${target}
            PRIVATE -Wall -pedantic -Wextra -Werror -Wno-unused-parameter
        )

        if(
            ${OPTIMIZER_BACKEND} STREQUAL "casadi"
            AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux"
            AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU"
        )
            target_compile_definitions(
                ${target}
                PRIVATE _GLIBCXX_USE_CXX11_ABI=0
            )
        endif()
    else()
        # Suppress the following warnings:
        #   * C4244: lossy conversion
        #   * C4251: missing dllexport/dllimport attribute on data member
        target_compile_options(${target} PRIVATE /wd4244 /wd4251 /WX)
    endif()

    target_compile_features(${target} PUBLIC cxx_std_20)
    if(MSVC)
        target_compile_options(${target} PUBLIC /bigobj)
    endif()
endmacro()
