cmake_minimum_required(VERSION 3.5.1)
project(geometry_common)

add_compile_options(-std=c++11)

# All warning enabled build option set to ON by default
option(ENABLE_ALL_WARNINGS "Build with all warnings enabled" ON)
if(ENABLE_ALL_WARNINGS)
    add_compile_options(-Wall)
endif(ENABLE_ALL_WARNINGS)

# Optimisation flag (O3) build option set to ON by default
option(BUILD_WITH_MAX_OPTIMISATION "Build with -O3 compile option" ON)
if(BUILD_WITH_MAX_OPTIMISATION)
    add_compile_options(-O3)
endif(BUILD_WITH_MAX_OPTIMISATION)

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

# The documentation build option set to OFF by default
option(BUILD_DOC "Build documentation" OFF)
if(BUILD_DOC)
    # check if Doxygen is installed
    find_package(Doxygen)
    if (DOXYGEN_FOUND)
        # set input and output files
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/doxygen/Doxyfile.in)
        set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

        # request to configure the file
        configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
        message("Doxygen build started")

        # note the option ALL which allows to build the docs together with the application
        add_custom_target( doc_doxygen ALL
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM )
    else (DOXYGEN_FOUND)
        message("Doxygen need to be installed to generate the doxygen documentation")
    endif (DOXYGEN_FOUND)
endif(BUILD_DOC)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# =========
# LIBRARIES
# =========
add_library(geometry_utils
    src/Utils.cpp
    # data structures
    src/Point2D.cpp
    src/Point3D.cpp
    src/Polygon2D.cpp
    src/Polyline2D.cpp
    src/Pose2D.cpp
    src/XYTheta.cpp
    src/Box.cpp
    src/LineSegment2D.cpp
    src/TransformMatrix2D.cpp
    src/TransformMatrix3D.cpp
)
target_link_libraries(geometry_utils
)

add_library(pointcloud_projector
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
        ${catkin_LIBRARIES}
    )
    catkin_add_gtest(pose_2d_test
        test/pose_2d_test.cpp
    )
    target_link_libraries(pose_2d_test
        geometry_utils
        ${catkin_LIBRARIES}
    )
    catkin_add_gtest(utils_test
        test/utils_test.cpp
    )
    target_link_libraries(utils_test
        geometry_utils
        ${catkin_LIBRARIES}
    )
    catkin_add_gtest(line_segment_2d_test
        test/line_segment_2d_test.cpp
    )
    target_link_libraries(line_segment_2d_test
        geometry_utils
        ${catkin_LIBRARIES}
    )
    catkin_add_gtest(polygon_2d_test
        test/polygon_2d_test.cpp
    )
    target_link_libraries(polygon_2d_test
        geometry_utils
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
