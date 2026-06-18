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

# Устанавливаем режим сборки по умолчанию
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Поиск зависимостей
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

# Общие include директории
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Сообщаем о найденных зависимостях
message(STATUS "Using Eigen3 from: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Using SDL2: ${SDL2_LIBRARIES}")
message(STATUS "Using SDL2_ttf: ${SDL2_TTF_LIBRARIES}")

# Подключаем поддиректории
add_subdirectory(src)

if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
