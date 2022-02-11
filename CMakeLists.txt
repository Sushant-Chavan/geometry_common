cmake_minimum_required(VERSION 2.8.3)
project(geometry_common)

add_compile_options(-std=c++11 -Wall)

if (WIN32)
    find_package(yaml-cpp REQUIRED)
else (WIN32)
    set(YAML_CPP_LIBRARIES yaml-cpp)
endif (WIN32)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
    visualization_msgs
    tf2_geometry_msgs
)

catkin_package(
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

# =========
# LIBRARIES
# =========
add_library(geometry_utils
    SHARED
        src/utils.cpp
        # data structures
        src/point.cpp
        src/pose_2d.cpp
        src/line_segment.cpp
)
target_link_libraries(geometry_utils
)

add_library(pointcloud_projector
    SHARED
        src/pointcloud_projector.cpp
)
target_link_libraries(pointcloud_projector
    geometry_utils
)

# =====
# TESTS
# =====
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(point_test
        test/point_test.cpp
    )
    target_link_libraries(point_test
        geometry_utils
    )
endif ()

