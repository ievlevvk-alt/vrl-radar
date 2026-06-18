# src/CMakeLists.txt

add_library(vrl_radar STATIC
    core/config.cpp
    core/config_loader.cpp
    processing/cluster.cpp
    processing/garbling_solver.cpp
    processing/kalman_filter.cpp
    processing/reply_processor.cpp
    processing/tracker.cpp
    simulation/simulator.cpp
    utils/logger.cpp
    utils/utils.cpp
)

target_link_libraries(vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    nlohmann_json  # Используем имя библиотеки, а не ::target
)

target_include_directories(vrl_radar
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(vrl_radar SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(vrl_radar
    PUBLIC
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
    PRIVATE
        $<$<CONFIG:Debug>:DEBUG>
)

set_property(GLOBAL PROPERTY ENABLE_VERBOSE_LOGGING ${ENABLE_VERBOSE_LOGGING})
