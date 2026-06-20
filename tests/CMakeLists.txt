# tests/CMakeLists.txt

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

set_target_properties(test_config_loader PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

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

# Тест для TrackerFilter
add_executable(test_tracker_filter
    test_tracker_filter.cpp
)

target_link_libraries(test_tracker_filter
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_tracker_filter
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_tracker_filter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_tracker_filter 
    COMMAND test_tracker_filter
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для Clusterer
add_executable(test_clusterer
    test_clusterer.cpp
)

target_link_libraries(test_clusterer
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_clusterer
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_clusterer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_clusterer 
    COMMAND test_clusterer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для TargetReport
add_executable(test_target_report
    test_target_report.cpp
)

target_link_libraries(test_target_report
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_target_report
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_target_report PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_target_report 
    COMMAND test_target_report
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для CircularHistory
add_executable(test_circular_history
    test_circular_history.cpp
)

target_link_libraries(test_circular_history
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_circular_history
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_circular_history PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_circular_history 
    COMMAND test_circular_history
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для ClusterCleanup
add_executable(test_cluster_cleanup
    test_cluster_cleanup.cpp
)

target_link_libraries(test_cluster_cleanup
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_cluster_cleanup
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_cluster_cleanup PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_cluster_cleanup 
    COMMAND test_cluster_cleanup
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для RangeGrouper
add_executable(test_range_grouper
    test_range_grouper.cpp
)

target_link_libraries(test_range_grouper
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_range_grouper
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_range_grouper PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_range_grouper 
    COMMAND test_range_grouper
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для ObjectPool
add_executable(test_object_pool
    test_object_pool.cpp
)

target_link_libraries(test_object_pool
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_object_pool
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_object_pool PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_object_pool 
    COMMAND test_object_pool
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для ExtendedKalmanFilter
add_executable(test_extended_kalman_filter
    test_extended_kalman_filter.cpp
)

target_link_libraries(test_extended_kalman_filter
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_extended_kalman_filter
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_extended_kalman_filter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_extended_kalman_filter 
    COMMAND test_extended_kalman_filter
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для UnscentedKalmanFilter
add_executable(test_unscented_kalman_filter
    test_unscented_kalman_filter.cpp
)

target_link_libraries(test_unscented_kalman_filter
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_unscented_kalman_filter
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_unscented_kalman_filter PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_unscented_kalman_filter 
    COMMAND test_unscented_kalman_filter
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# Тест для FilterSelection
add_executable(test_filter_selection
    test_filter_selection.cpp
)

target_link_libraries(test_filter_selection
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_filter_selection
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_filter_selection PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_filter_selection 
    COMMAND test_filter_selection
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# НОВЫЙ ТЕСТ: DBSCANClusterer
add_executable(test_dbscan_clusterer
    test_dbscan_clusterer.cpp
)

target_link_libraries(test_dbscan_clusterer
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_dbscan_clusterer
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_dbscan_clusterer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_dbscan_clusterer 
    COMMAND test_dbscan_clusterer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

# НОВЫЙ ТЕСТ: PointBuffer
add_executable(test_point_buffer
    test_point_buffer.cpp
)

target_link_libraries(test_point_buffer
    vrl_radar
    GTest::gtest_main
)

target_compile_definitions(test_point_buffer
    PRIVATE
        $<$<BOOL:${ENABLE_VERBOSE_LOGGING}>:ENABLE_VERBOSE_LOGGING>
)

set_target_properties(test_point_buffer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests"
)

add_test(NAME test_point_buffer 
    COMMAND test_point_buffer
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

