# file: tests/CMakeLists.txt
# Тесты для библиотеки radar_sim

# Тест для types
add_executable(test_types test_types.cpp)
target_link_libraries(test_types PRIVATE radar_sim)
add_test(NAME TypesTest COMMAND test_types)

# Тест для simulator
add_executable(test_simulator test_simulator.cpp)
target_link_libraries(test_simulator PRIVATE radar_sim)
add_test(NAME SimulatorTest COMMAND test_simulator)

# Тест для grouping (пост-обработка оборота)
add_executable(test_grouping test_grouping.cpp)
target_link_libraries(test_grouping PRIVATE radar_sim)
add_test(NAME GroupingTest COMMAND test_grouping)

# Тест для cluster_tracker
add_executable(test_cluster_tracker test_cluster_tracker.cpp)
target_link_libraries(test_cluster_tracker PRIVATE radar_sim)
add_test(NAME ClusterTrackerTest COMMAND test_cluster_tracker)

# Тест для cluster_processor
add_executable(test_cluster_processor test_cluster_processor.cpp)
target_link_libraries(test_cluster_processor PRIVATE radar_sim)
add_test(NAME ClusterProcessorTest COMMAND test_cluster_processor)

# file: tests/CMakeLists.txt (дополнение)

# Тест для garbling solver
add_executable(test_garbling_solver test_garbling_solver.cpp)
target_link_libraries(test_garbling_solver PRIVATE radar_sim)
add_test(NAME GarblingSolverTest COMMAND test_garbling_solver)