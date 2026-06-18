# VRL-Radar/tools/CMakeLists.txt

# Этап 1: Генерация ответов
add_executable(1_generate_replies 1_generate_replies.cpp)
target_link_libraries(1_generate_replies vrl_radar)

# Этап 2+3: Объединённая обработка
add_executable(2_3_combined 2_3_combined.cpp)
target_link_libraries(2_3_combined vrl_radar)

# Этап 4: Плеер с отображением
add_executable(4_radar_player 4_radar_player.cpp)
target_link_libraries(4_radar_player 
    vrl_radar
    ${SDL2_LIBRARIES}
    ${SDL2_TTF_LIBRARIES}
)

# Устанавливаем утилиты
install(TARGETS 1_generate_replies 2_3_combined 4_radar_player
    RUNTIME DESTINATION bin
)
