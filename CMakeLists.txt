# Set default build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(WARNING "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

cmake_minimum_required(VERSION 3.10)
project(game LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set PKG_CONFIG_PATH environment variable clearly for pkg-config
set(ENV{PKG_CONFIG_PATH}
    "$ENV{HOME}/opt/SDL3/lib/pkgconfig:$ENV{HOME}/opt/SDL3_ttf/lib/pkgconfig:$ENV{HOME}/opt/SDL3_image/lib/pkgconfig"
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL3 REQUIRED sdl3)
pkg_check_modules(SDL3_TTF REQUIRED sdl3-ttf)
pkg_check_modules(SDL3_IMAGE REQUIRED sdl3-image)


# Common settings
set(COMMON_INCLUDES
    ${SDL3_INCLUDE_DIRS}
    ${SDL3_TTF_INCLUDE_DIRS}
    ${SDL3_IMAGE_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/include
)

set(COMMON_COMPILE_OPTS
    ${SDL3_CFLAGS_OTHER}
    ${SDL3_TTF_CFLAGS_OTHER}
    ${SDL3_IMAGE_CFLAGS_OTHER}
)
set(COMMON_LINK_DIRS
    ${SDL3_LIBRARY_DIRS}
    ${SDL3_TTF_LIBRARY_DIRS}
    ${SDL3_IMAGE_LIBRARY_DIRS}
)
set(COMMON_LIBS
    ${SDL3_LIBRARIES}
    ${SDL3_TTF_LIBRARIES}
    ${SDL3_IMAGE_LIBRARIES}
)

set(CMAKE_CXX_FLAGS_DEBUG "-g")

# --- IMGUI integration ---
find_package(OpenGL REQUIRED)
list(APPEND COMMON_LIBS OpenGL::GL)

# IMGUI
# set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/imgui)

# file(GLOB IMGUI_SRC
#     ${IMGUI_DIR}/*.cpp
#     ${IMGUI_DIR}/backends/imgui_impl_sdl3.cpp
#     ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
# )

# add_library(imgui STATIC ${IMGUI_SRC})
# target_include_directories(imgui PUBLIC
#     ${IMGUI_DIR}
#     ${IMGUI_DIR}/backends
#     ${COMMON_INCLUDES}
# )
# target_compile_options(imgui PUBLIC ${COMMON_COMPILE_OPTS})

# 1) Build your code as a static library
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(FILTER ALL_SRC EXCLUDE REGEX ".*/src/main\\.cpp$")
set(GAME_SRC ${ALL_SRC})

add_library(game_lib STATIC ${GAME_SRC})
target_include_directories(game_lib PUBLIC ${COMMON_INCLUDES})
target_compile_options(game_lib PUBLIC ${COMMON_COMPILE_OPTS})

# 2) Main executable just needs main.cpp + link to game_lib
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_directories(${PROJECT_NAME} PRIVATE ${COMMON_LINK_DIRS})
target_link_libraries(${PROJECT_NAME}
    # PRIVATE game_lib imgui ${COMMON_LIBS}
    PRIVATE game_lib ${COMMON_LIBS}
)

# 3) Tests: one executable per test, also link game_lib
file(GLOB TEST_SOURCES "${CMAKE_SOURCE_DIR}/tests/*.cpp")
foreach(test_src IN LISTS TEST_SOURCES)
  get_filename_component(test_name ${test_src} NAME_WE)

  add_executable(${test_name} ${test_src})
  target_link_directories(${test_name} PRIVATE ${COMMON_LINK_DIRS})
  target_link_libraries(${test_name}
    PRIVATE game_lib ${COMMON_LIBS}
  )
endforeach()

