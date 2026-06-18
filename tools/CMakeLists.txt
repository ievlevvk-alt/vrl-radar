# tools/CMakeLists.txt

get_property(ENABLE_VERBOSE_LOGGING GLOBAL PROPERTY ENABLE_VERBOSE_LOGGING)

# Утилита миграции конфигурации
add_executable(migrate_config
    migrate_config.cpp
)

target_link_libraries(migrate_config
    vrl_radar
    nlohmann_json
)

target_compile_definitions(migrate_config
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

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

# Утилита 2-3
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

# Утилита 4
add_executable(4_radar_player
    4_radar_player.cpp
)

target_link_libraries(4_radar_player
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

target_compile_definitions(4_radar_player
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(migrate_config 1_generate_replies 2_3_combined 4_radar_player
    PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tools"
)
