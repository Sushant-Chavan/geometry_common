cmake_minimum_required(VERSION 2.8.3)
project(geometry_common)

add_compile_options(-std=c++11 -Wall -O3)

if (WIN32)
    find_package(yaml-cpp REQUIRED)
else (WIN32)
    set(YAML_CPP_LIBRARIES yaml-cpp)
endif (WIN32)

find_package(catkin REQUIRED COMPONENTS
    tf
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS
        tf
        std_msgs
        sensor_msgs
        geometry_msgs
        nav_msgs
        visualization_msgs
        tf2_geometry_msgs
    INCLUDE_DIRS
        include
    LIBRARIES
        geometry_utils
        pointcloud_projector
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
        src/Utils.cpp
        # data structures
        src/Point2D.cpp
        src/Point3D.cpp
        src/Polygon2D.cpp
        src/Polyline2D.cpp
        src/Pose2D.cpp
        src/Box.cpp
        src/LineSegment2D.cpp
)
target_link_libraries(geometry_utils
)

add_library(pointcloud_projector
    SHARED
        src/PointCloudProjector.cpp
)
target_link_libraries(pointcloud_projector
    geometry_utils
)

# =====
# TESTS
# =====
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(point_3d_test
        test/point_3d_test.cpp
    )
    target_link_libraries(point_3d_test
        geometry_utils
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    catkin_add_gtest(pose_2d_test
        test/pose_2d_test.cpp
    )
    target_link_libraries(pose_2d_test
        geometry_utils
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
    )
    catkin_add_gtest(utils_test
        test/utils_test.cpp
    )
    target_link_libraries(utils_test
        geometry_utils
        ${YAML_CPP_LIBRARIES}
        ${catkin_LIBRARIES}
    )
endif ()

# =======
# INSTALL
# =======
install(TARGETS geometry_utils pointcloud_projector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
