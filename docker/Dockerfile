ARG TAG
FROM ghcr.io/tesseract-robotics/tesseract_qt:${TAG}

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install ROS
ARG ROS_DISTRO

# Add the ROS sources
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt update -qq

RUN apt install -y \
  ros-${ROS_DISTRO}-ros-base \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-ros2launch \
  ros-${ROS_DISTRO}-launch-ros \
  ros-${ROS_DISTRO}-launch-xml \
  ros-${ROS_DISTRO}-launch-yaml

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/tesseract_ros2

# Install the latest version of octomap_msgs for Humble and beyond
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract_ros2 \
  if [[ "${ROS_DISTRO}" > "foxy" ]]; then \
  cd ${WORKSPACE_DIR} \
  && vcs import src < src/tesseract_ros2/.github/workflows/dependencies.repos --shallow ; \
  fi

RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract_ros2 \
  source /opt/ros/${ROS_DISTRO}/setup.bash \
  && source /opt/tesseract_qt/install/setup.bash \
  && rosdep update --include-eol-distros \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry --skip-keys=osqp-eigen

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/tesseract_ros2 \
  source /opt/tesseract_qt/install/setup.bash \
  && source /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd ${WORKSPACE_DIR} \ 
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log
