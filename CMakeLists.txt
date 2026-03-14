cmake_minimum_required(VERSION 3.10)
project(game LANGUAGES C CXX)

# tell CMake to output a compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(WARNING "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

# CTest
enable_testing()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set PKG_CONFIG_PATH environment variable clearly for pkg-config
# set(ENV{PKG_CONFIG_PATH}
#     "$ENV{HOME}/opt/SDL3/lib64/pkgconfig:$ENV{HOME}/opt/SDL3_ttf/lib64/pkgconfig:$ENV{HOME}/opt/SDL3_image/lib64/pkgconfig"
# )
# Prefer this
list(APPEND CMAKE_PREFIX_PATH
    "$ENV{HOME}/opt/SDL3"
    "$ENV{HOME}/opt/SDL3_ttf"
    "$ENV{HOME}/opt/SDL3_image"
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL3 REQUIRED sdl3)
pkg_check_modules(SDL3_TTF REQUIRED sdl3-ttf)
pkg_check_modules(SDL3_IMAGE REQUIRED sdl3-image)

set(VENDOR_DIR ${CMAKE_SOURCE_DIR}/vendor)
set(IMGUI_DIR ${VENDOR_DIR}/imgui)
set(IMPLOT_DIR ${VENDOR_DIR}/implot)

# non-buildable target: no sources, output, just usage reqs
add_library(common_deps INTERFACE)


target_include_directories(common_deps INTERFACE
    ${SDL3_INCLUDE_DIRS}
    ${SDL3_TTF_INCLUDE_DIRS}
    ${SDL3_IMAGE_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)
# file(GLOB_RECURSE GAME_HEADERS "${CMAKE_SOURCE_DIR}/include/*.h")

target_link_directories(common_deps INTERFACE
    ${SDL3_LIBRARY_DIRS}
    ${SDL3_TTF_LIBRARY_DIRS}
    ${SDL3_IMAGE_LIBRARY_DIRS}
)

target_compile_options(common_deps INTERFACE
    ${SDL3_CFLAGS_OTHER}
    ${SDL3_TTF_CFLAGS_OTHER}
    ${SDL3_IMAGE_CFLAGS_OTHER}
)

target_link_libraries(common_deps INTERFACE
    ${SDL3_LIBRARIES}
    ${SDL3_TTF_LIBRARIES}
    ${SDL3_IMAGE_LIBRARIES}
)

# imgui

file(GLOB IMGUI_SRC
     ${IMGUI_DIR}/*.cpp
     ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
     ${IMGUI_DIR}/backends/imgui_impl_sdlrenderer3.cpp
)

add_library(imgui STATIC ${IMGUI_SRC})
target_include_directories(imgui PUBLIC
     ${IMGUI_DIR}
     ${IMGUI_DIR}/backends
     ${SDL3_INCLUDE_DIRS}
)


# --- IMPLOT integration ---
file(GLOB IMPLOT_SRC
     ${IMPLOT_DIR}/implot.cpp
     ${IMPLOT_DIR}/implot_items.cpp
)

add_library(implot STATIC ${IMPLOT_SRC})
target_include_directories(implot PUBLIC 
    ${IMPLOT_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
target_link_libraries(implot PUBLIC imgui)

set(CORE_SRC
    core/src/sim_core.c
  )

# ADD CORE - pure C library
add_library(core_lib STATIC ${CORE_SRC})
# make core_lib export its math dependency
target_link_libraries(core_lib PUBLIC m)

target_include_directories(core_lib
    PUBLIC
        ${CMAKE_SOURCE_DIR}/core/include
)

# Enforce strict C99 for the core only
set_target_properties(core_lib PROPERTIES
    LINKER_LANGUAGE C
    C_STANDARD 99
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

# Warnings for C core (only applied to C files)
target_compile_options(core_lib PRIVATE
  $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra -Wpedantic -Werror -Wshadow -Wdouble-promotion>
  $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Werror -Woverloaded-virtual> # should never trigger
)


# 1) Build your code as a static library
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(FILTER ALL_SRC EXCLUDE REGEX ".*/src/main\\.cpp$")
set(GAME_SRC ${ALL_SRC})

add_library(game_lib STATIC ${GAME_SRC})
target_link_libraries(game_lib
  PUBLIC
    core_lib
    common_deps
    imgui
    implot
)

# 2) Main executable just needs main.cpp + link to game_lib
add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    game_lib
)


# copy resources into build dir so they are at the level that executable is
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/resources
    $<TARGET_FILE_DIR:${PROJECT_NAME}>/resources
)



# 3) Tests: one executable per test, also link game_lib
# # -------------------------
# C++ tests (integration)
# -------------------------
file(GLOB TEST_SOURCES "${CMAKE_SOURCE_DIR}/tests/*.cpp")
foreach(test_src IN LISTS TEST_SOURCES)
  get_filename_component(test_name ${test_src} NAME_WE)

  add_executable(${test_name} ${test_src})
  # target_link_libraries(${test_name}
  target_link_libraries(${test_name}
    PRIVATE
      game_lib
      common_deps
  )
  add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# -------------------------
# C tests (core-only)
# -------------------------
file(GLOB TEST_C_SOURCES
    "${CMAKE_SOURCE_DIR}/tests/core/*.c"
)

foreach(test_src IN LISTS TEST_C_SOURCES)
    get_filename_component(test_name ${test_src} NAME_WE)

    add_executable(${test_name} ${test_src})

    target_include_directories(${test_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/core/include
    )

    target_link_libraries(${test_name}
        PRIVATE core_lib
    )

    # Enforce strict C99 on tests too
    set_target_properties(${test_name} PROPERTIES
        LINKER_LANGUAGE C
        C_STANDARD 99
        C_STANDARD_REQUIRED YES
        C_EXTENSIONS NO
    )

    target_compile_options(${test_name} PRIVATE
        -Wall -Wextra -Wpedantic
    )

    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()

# add_custom_target(run_tests
#     COMMAND ctest --output-on-failure
#   )

add_custom_target(run_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure --build-and-test .
    DEPENDS ${test_targets}  # collect these in your foreach
)
