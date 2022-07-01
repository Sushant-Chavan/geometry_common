FROM ros:melodic

SHELL [ "/bin/bash", "-c" ]

# Install dependencies
RUN apt-get update -qq && \
    apt-get install -yqq sudo \
                         python-catkin-tools \
                         ros-$ROS_DISTRO-catkin \
                         ros-$ROS_DISTRO-tf \
                         ros-$ROS_DISTRO-tf2-geometry-msgs && \
    apt-get clean

# Copy the geometry_common source code to the docker container
WORKDIR /workspace/catkin_ws/src/geometry_common
ADD . /workspace/catkin_ws/src/geometry_common/

# Compile the ROS catkin workspace
RUN cd /workspace/catkin_ws && \
    /ros_entrypoint.sh catkin build --limit-status-rate 0.001

# Run unit tests
RUN source /workspace/catkin_ws/devel/setup.bash && \
    cd /workspace/catkin_ws/src/geometry_common && \
    /ros_entrypoint.sh catkin build --this --limit-status-rate 0.001 --catkin-make-args run_tests -- && \
    rosrun geometry_common geometry_common_test
