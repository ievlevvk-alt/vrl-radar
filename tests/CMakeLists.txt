# tests/CMakeLists.txt

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

gtest_discover_tests(test_config_loader)

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

gtest_discover_tests(test_kalman_filter)

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

gtest_discover_tests(test_cluster_tracker)

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

gtest_discover_tests(test_reply_processor)
