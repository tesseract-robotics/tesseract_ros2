ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract_qt:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install ROS
ARG ROS_DISTRO
RUN apt install -y \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-ros2launch \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-launch-xml \
  ros-${ROS_DISTRO}-launch-yaml

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/tesseract_ros2
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract_ros2 \
  source /opt/ros/${ROS_DISTRO}/setup.bash \
  && source /opt/tesseract_qt/install/setup.bash \
  && rosdep update --include-eol-distros \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract_ros2 \
  source /opt/tesseract_qt/install/setup.bash \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log
