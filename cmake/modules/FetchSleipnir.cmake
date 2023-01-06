macro(fetch_sleipnir)
  include(FetchContent)

  FetchContent_Declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
    GIT_TAG c34dc73aa57b722c6ec5dcb89df03275010bd2e8
  )

  FetchContent_GetProperties(Sleipnir)
  if(NOT Sleipnir_POPULATED)
    FetchContent_Populate(Sleipnir)
    add_subdirectory(${sleipnir_SOURCE_DIR} ${sleipnir_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
endmacro()
