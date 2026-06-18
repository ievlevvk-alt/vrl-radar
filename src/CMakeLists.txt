# src/CMakeLists.txt

# Библиотека VRL Radar
add_library(vrl_radar STATIC
    core/config.cpp
    processing/cluster.cpp
    processing/garbling_solver.cpp
    processing/kalman_filter.cpp
    processing/reply_processor.cpp
    processing/tracker.cpp
    simulation/simulator.cpp
    utils/logger.cpp
    utils/utils.cpp
)

# Подключаем зависимости
target_link_libraries(vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
    ${EIGEN3_LIBRARIES}
)

# Устанавливаем include директории
target_include_directories(vrl_radar
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(vrl_radar SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

# Добавляем опции компиляции
target_compile_definitions(vrl_radar
    PUBLIC
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
    PRIVATE
        $<$<CONFIG:Debug>:DEBUG>
)

# Передаем опции в инструменты
set_property(GLOBAL PROPERTY ENABLE_VERBOSE_LOGGING ${ENABLE_VERBOSE_LOGGING})
