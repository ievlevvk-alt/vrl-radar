# tools/CMakeLists.txt

get_property(ENABLE_VERBOSE_LOGGING GLOBAL PROPERTY ENABLE_VERBOSE_LOGGING)

# Утилита 1
add_executable(1_generate_replies
    1_generate_replies.cpp
)

target_link_libraries(1_generate_replies
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(1_generate_replies
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# Утилита 2-3 (старая версия)
add_executable(2_3_combined
    2_3_combined.cpp
)

target_link_libraries(2_3_combined
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(2_3_combined
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# Утилита 2-3 V2
add_executable(2_3_combined_v2
    2_3_combined_v2.cpp
)

target_link_libraries(2_3_combined_v2
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(2_3_combined_v2
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# Утилита 2-3 V2 Streaming
add_executable(2_3_combined_v2_streaming
    2_3_combined_v2_streaming.cpp
)

target_link_libraries(2_3_combined_v2_streaming
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(2_3_combined_v2_streaming
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# ========================================================================
# НОВАЯ УТИЛИТА - MANUAL CONTROL
# ========================================================================

add_executable(2_3_combined_v2_manual
    2_3_combined_v2_manual.cpp
)

target_link_libraries(2_3_combined_v2_manual
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(2_3_combined_v2_manual
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# Утилита 4 - Radar Player
add_executable(4_radar_player
    4_radar_player.cpp
)

target_link_libraries(4_radar_player
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(4_radar_player
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# Устанавливаем выходные директории
set_target_properties(1_generate_replies 2_3_combined 2_3_combined_v2 
    2_3_combined_v2_streaming 2_3_combined_v2_manual 4_radar_player
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)
