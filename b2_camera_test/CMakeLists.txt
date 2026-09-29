cmake_minimum_required(VERSION 3.5)
project(b2_camera_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 不编译 Unitree 官方 examples
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

# 引入本机的 Unitree SDK2
add_subdirectory(
    /home/jetson/unitree_sdk2
    ${CMAKE_BINARY_DIR}/unitree_sdk2
)

add_executable(
    b2_front_camera_test
    b2_front_camera_test.cpp
)

target_link_libraries(
    b2_front_camera_test
    unitree_sdk2
)

add_executable(
    b2_camera_fps_test
    b2_camera_fps_test.cpp
)

target_link_libraries(
    b2_camera_fps_test
    unitree_sdk2
)
