# file: CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(RadarSimulator VERSION 0.1.0 LANGUAGES CXX)

# Требуем C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Опции сборки
option(BUILD_TESTS "Build tests" ON)
option(BUILD_TOOLS "Build tools" ON)

# Устанавливаем режим сборки по умолчанию
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
endif()

# Путь к Eigen
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
message(STATUS "Using Eigen3 from: ${EIGEN3_INCLUDE_DIR}")

# Пути для include
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Основная библиотека - добавили reply_processor.cpp
add_library(radar_sim
    src/simulator.cpp
    src/utils.cpp
    src/grouping.cpp
    src/cluster_processor.cpp
    src/radar_system.cpp
    src/garbling_solver.cpp
    src/kalman_filter.cpp
    src/track_manager.cpp
    src/reply_processor.cpp      # <--- ДОБАВИТЬ
)

# Подключаем тесты
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Подключаем инструменты
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()
