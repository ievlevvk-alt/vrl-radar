# file: tools/CMakeLists.txt
add_executable(radar_processor radar_processor.cpp)
target_link_libraries(radar_processor PRIVATE radar_sim)

# Копируем пример конфигурации в бинарную директорию
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../radar.conf
    ${CMAKE_CURRENT_BINARY_DIR}/radar.conf COPYONLY
)