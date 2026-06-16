# file: tools/CMakeLists.txt

# Консольный процессор
add_executable(radar_cli radar_processor.cpp)
target_link_libraries(radar_cli PRIVATE radar_sim)

# Простой визуализатор
find_package(SDL2 REQUIRED)
add_executable(radar_viewer_simple radar_viewer_simple.cpp)
target_link_libraries(radar_viewer_simple PRIVATE radar_sim ${SDL2_LIBRARIES})
target_include_directories(radar_viewer_simple PRIVATE ${SDL2_INCLUDE_DIRS})


# GUI визуализатор
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 sdl2)
pkg_check_modules(SDL2_TTF SDL2_ttf)

if(SDL2_FOUND AND SDL2_TTF_FOUND)
    add_executable(radar_viewer radar_viewer.cpp)
    target_include_directories(radar_viewer PRIVATE ${SDL2_INCLUDE_DIRS} ${SDL2_TTF_INCLUDE_DIRS})
    target_link_libraries(radar_viewer PRIVATE 
        radar_sim 
        ${SDL2_LIBRARIES} 
        ${SDL2_TTF_LIBRARIES}
    )
    message(STATUS "Building radar_viewer with SDL2 support")
else()
    message(STATUS "SDL2 or SDL2_ttf not found - radar_viewer disabled")
endif()

# Копируем конфигурацию
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../radar.conf
    ${CMAKE_CURRENT_BINARY_DIR}/radar.conf COPYONLY
)
