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
