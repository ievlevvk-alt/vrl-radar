# VRL-Radar/src/CMakeLists.txt

# Основная библиотека
add_library(vrl_radar
    ${CMAKE_CURRENT_SOURCE_DIR}/core/config.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/tracker.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/kalman_filter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/cluster.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/garbling_solver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/garbling_solver_threshold.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/garbling_solver_iterative.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/processing/reply_processor.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/simulation/simulator.cpp
    
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/utils.cpp
)

# Настройка include директорий для библиотеки
target_include_directories(vrl_radar
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Подключаем зависимости
target_link_libraries(vrl_radar
    PUBLIC
        ${EIGEN3_LIBRARIES}
)

# Устанавливаем библиотеку
install(TARGETS vrl_radar
    EXPORT VRLRadarTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/vrl
    DESTINATION include
)
