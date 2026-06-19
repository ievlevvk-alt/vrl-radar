# tests/CMakeLists.txt

# Включаем поддержку GTest
include(GoogleTest)

# Тест для ConfigLoader
add_executable(test_config_loader
    test_config_loader.cpp
)

target_link_libraries(test_config_loader
    vrl_radar
    GTest::gtest_main
    nlohmann_json
)

target_compile_definitions(test_config_loader
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

# Явно указываем выходную директорию
set_target_properties(test_config_loader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

# Регистрируем тест в CTest
add_test(NAME test_config_loader 
    COMMAND test_config_loader
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для Kalman Filter
add_executable(test_kalman_filter
    test_kalman_filter.cpp
)

target_link_libraries(test_kalman_filter
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_kalman_filter
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_kalman_filter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_kalman_filter 
    COMMAND test_kalman_filter
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для ClusterTracker
add_executable(test_cluster_tracker
    test_cluster_tracker.cpp
)

target_link_libraries(test_cluster_tracker
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_cluster_tracker
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_cluster_tracker PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_cluster_tracker 
    COMMAND test_cluster_tracker
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для ReplyProcessor
add_executable(test_reply_processor
    test_reply_processor.cpp
)

target_link_libraries(test_reply_processor
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_reply_processor
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_reply_processor PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_reply_processor 
    COMMAND test_reply_processor
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)
