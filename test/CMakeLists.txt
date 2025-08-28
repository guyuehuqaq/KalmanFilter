cmake_minimum_required(VERSION 3.5)

find_package(GTest CONFIG REQUIRED)
add_executable(kalman_test kalman_test.cc)
target_link_libraries(kalman_test moxun_track_kalman GTest::gtest GTest::gtest_main)