macro(fetch_sleipnir)
  include(FetchContent)

  FetchContent_Declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir.git
    GIT_TAG 0d20a9f712f1e997f734038a977392d8bc6a19f4
  )

  FetchContent_GetProperties(Sleipnir)
  if(NOT Sleipnir_POPULATED)
    FetchContent_Populate(Sleipnir)
    add_subdirectory(${sleipnir_SOURCE_DIR} ${sleipnir_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
endmacro()
