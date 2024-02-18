macro(trajoptlib_compiler_flags target)
    if(NOT MSVC)
        target_compile_options(
            ${target}
            PRIVATE -Wall -pedantic -Wextra -Werror -Wno-unused-parameter
        )

        # clang 18 warns on `operator"" _a` in dependencies
        if(
            ${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang"
            AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_GREATER_EQUAL "18"
        )
            target_compile_options(
                ${target}
                PRIVATE -Wno-deprecated-literal-operator
            )
        endif()

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
        target_compile_options(
            ${target}
            PRIVATE /wd4146 /wd4244 /wd4251 /wd4267 /WX
        )
    endif()

    target_compile_features(${target} PUBLIC cxx_std_20)
    if(MSVC)
        target_compile_options(${target} PUBLIC /bigobj)
    endif()
endmacro()
