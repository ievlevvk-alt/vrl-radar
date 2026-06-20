# src/CMakeLists.txt

add_library(vrl_radar STATIC
    core/config.cpp
    core/config_loader.cpp
    core/point_buffer.cpp           # <-- НОВЫЙ ФАЙЛ
    core/cluster.cpp           # <-- НОВЫЙ ФАЙЛ
    processing/cluster.cpp
    processing/dbscan_clusterer.cpp
    processing/garbling_solver.cpp
    processing/kalman_filter.cpp
    processing/extended_kalman_filter.cpp
    processing/unscented_kalman_filter.cpp
    processing/legacy_clusterer.cpp
    processing/range_grouper.cpp
    processing/rbs_processor.cpp
    processing/uvd_processor.cpp
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
    nlohmann_json
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
        $<$<BOOL:${ENABLE_TRACE_LOGGING}>:VRL_ENABLE_TRACE_LOGGING=1>
        $<$<BOOL:${ENABLE_DEBUG_LOGGING}>:VRL_ENABLE_DEBUG_LOGGING=1>
        $<$<NOT:$<BOOL:${ENABLE_DEBUG_LOGGING}>>:VRL_ENABLE_DEBUG_LOGGING=0>
    PRIVATE
        $<$<CONFIG:Debug>:CMAKE_BUILD_TYPE_DEBUG>
)

set_property(GLOBAL PROPERTY ENABLE_VERBOSE_LOGGING ${ENABLE_VERBOSE_LOGGING})
