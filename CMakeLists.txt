# VRL-Radar/CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(VRLRadar VERSION 0.2.0 LANGUAGES C CXX)

# Требуем C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Опции сборки
option(BUILD_TOOLS "Build tools" ON)
option(BUILD_TESTS "Build tests" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)
option(ENABLE_VERBOSE_LOGGING "Enable verbose debug logging" OFF)
option(ENABLE_TRACE_LOGGING "Enable TRACE level logging (very verbose)" OFF)
option(ENABLE_DEBUG_LOGGING "Enable DEBUG level logging" ON)

# Новая опция для размера буфера точек
set(POINT_BUFFER_SIZE 65536 CACHE STRING "Size of point buffer for storage")

# Устанавливаем режим сборки по умолчанию
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Определяем, Debug ли сборка (для логирования)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(CMAKE_BUILD_TYPE_DEBUG)
endif()

# ============================================================================
# ЗАВИСИМОСТИ
# ============================================================================

# Eigen3
find_package(Eigen3 REQUIRED)

# SDL2
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

# nlohmann/json
find_path(NLOHMANN_INCLUDE_DIR
    NAMES nlohmann/json.hpp
    PATHS
        /usr/include
        /usr/local/include
        /usr/include/nlohmann
        /usr/local/include/nlohmann
)

if(NLOHMANN_INCLUDE_DIR)
    message(STATUS "Found nlohmann/json in: ${NLOHMANN_INCLUDE_DIR}")
    add_library(nlohmann_json INTERFACE)
    target_include_directories(nlohmann_json INTERFACE ${NLOHMANN_INCLUDE_DIR})
    set(NLOHMANN_FOUND TRUE)
else()
    message(FATAL_ERROR "nlohmann/json not found. Please install: sudo apt-get install nlohmann-json3-dev")
endif()

# Google Test
if(BUILD_TESTS)
    find_package(GTest REQUIRED)
    include(GoogleTest)
    enable_testing()
endif()

# ============================================================================
# КОНФИГУРАЦИЯ СБОРКИ
# ============================================================================

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "Using Eigen3 from: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Using SDL2: ${SDL2_LIBRARIES}")
message(STATUS "Using SDL2_ttf: ${SDL2_TTF_LIBRARIES}")
message(STATUS "Using nlohmann/json: ${NLOHMANN_INCLUDE_DIR}")

# Передаем опции (без DEBUG макроса)
add_compile_definitions(
    $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
    $<$<BOOL:${ENABLE_TRACE_LOGGING}>:VRL_ENABLE_TRACE_LOGGING=1>
    $<$<BOOL:${ENABLE_DEBUG_LOGGING}>:VRL_ENABLE_DEBUG_LOGGING=1>
    $<$<NOT:$<BOOL:${ENABLE_DEBUG_LOGGING}>>:VRL_ENABLE_DEBUG_LOGGING=0>
)

# ============================================================================
# ПОДДИРЕКТОРИИ
# ============================================================================

add_subdirectory(src)

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ============================================================================
# ИТОГОВАЯ ИНФОРМАЦИЯ
# ============================================================================
message(STATUS "")
message(STATUS "=== VRL-Radar Build Configuration ===")
message(STATUS "Build type:          ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ Standard:        ${CMAKE_CXX_STANDARD}")
message(STATUS "Build tools:         ${BUILD_TOOLS}")
message(STATUS "Build tests:         ${BUILD_TESTS}")
message(STATUS "Build examples:      ${BUILD_EXAMPLES}")
message(STATUS "Verbose logging:     ${ENABLE_VERBOSE_LOGGING}")
message(STATUS "Trace logging:       ${ENABLE_TRACE_LOGGING}")
message(STATUS "Debug logging:       ${ENABLE_DEBUG_LOGGING}")
message(STATUS "=======================================")
message(STATUS "")
