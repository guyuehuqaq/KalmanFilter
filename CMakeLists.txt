cmake_minimum_required(VERSION 3.5)
PROJECT(moxun_track_kalman C CXX ASM)

file(GLOB kalman_srcs "*.cpp" "*.cc" "*.h")

if (WIN32)
    add_library(moxun_track_kalman STATIC
            ${kalman_srcs}
    )
else ()
    add_library(moxun_track_kalman SHARED
            ${kalman_srcs}
            kf_modules.h
    )
endif ()

target_link_libraries(moxun_track_kalman
        moxun_track_basic
)

target_include_directories(moxun_track_kalman PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
)

if (USE_TEST)
    add_subdirectory(test)
endif ()