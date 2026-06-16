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

# Поиск Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Using Eigen3 from: ${EIGEN3_INCLUDE_DIR}")

# Поиск SDL2 через pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

# Пути для include
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${SDL2_TTF_INCLUDE_DIRS})

# Основная библиотека
add_library(radar_sim
    src/simulator.cpp
    src/utils.cpp
    src/grouping.cpp
    src/cluster_processor.cpp
    src/radar_system.cpp
    src/garbling_solver.cpp
    src/kalman_filter.cpp
    src/track_manager.cpp
    src/reply_processor.cpp
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

# === НОВЫЕ УТИЛИТЫ ===

# Этап 1: Генерация ответов
add_executable(1_generate_replies tools/1_generate_replies.cpp)
target_link_libraries(1_generate_replies radar_sim)

# Этап 2: Формирование плотов
add_executable(2_form_plots tools/2_form_plots.cpp)
target_link_libraries(2_form_plots radar_sim)

# Этап 3: Трековая обработка
add_executable(3_track_processing tools/3_track_processing.cpp)
target_link_libraries(3_track_processing radar_sim)

# Этап 4: Плеер с отображением
add_executable(4_radar_player tools/4_radar_player.cpp)
target_link_libraries(4_radar_player 
    radar_sim 
    ${SDL2_LIBRARIES} 
    ${SDL2_TTF_LIBRARIES}
)
