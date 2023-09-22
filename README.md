# Tesseract ROS

Platform             | CI Status
---------------------|:---------
Ubuntu               | [![Build Status](https://github.com/tesseract-robotics/tesseract_ros2/workflows/Ubuntu/badge.svg)](https://github.com/tesseract-robotics/tesseract_ros2/actions)
Lint  (Clang-Format) | [![Build Status](https://github.com/tesseract-robotics/tesseract_ros2/workflows/Clang-Format/badge.svg)](https://github.com/tesseract-robotics/tesseract_ros2/actions)
Lint  (Clang Tidy)   | [![Build Status](https://github.com/tesseract-robotics/tesseract_ros2/workflows/Clang-Tidy/badge.svg)](https://github.com/tesseract-robotics/tesseract_ros2/actions)

### Supported ROS Distros
| Distro         | Support |
| :---           | :---:   |
| ROS Foxy       | &check; |
| ROS Humble     | &check; |

[![Github Issues](https://img.shields.io/github/issues/ros-industrial-consortium/tesseract_ros.svg)](http://github.com/ros-industrial-consortium/tesseract_ros/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

## Package Descriptions

### `tesseract_ros_examples`
This package contains ROS examples using tesseract and tesseract_ros for motion planning and collision checking.

### `tesseract_rosutils`
This package contains the utilities like converting from ROS message types to native Tesseract types and the reverse.

### `tesseract_msgs`
This package contains the ROS message types used by Tesseract ROS.

### `tesseract_rviz`
This package contains the ROS visualization plugins for Rviz to visualize Tesseract.
All of the features have been composed in libraries to enable to the ability to create custom displays quickly.

### `tesseract_monitoring`
This package contains different types of environment monitors.
It currently contains a contact monitor and environment monitor.
The contact monitor will monitor the active environment state and publish contact information.
This is useful if the robot is being controlled outside of ROS, but you want to make sure it does not collide with objects in the environment.
The second is the environment monitor, which is the main environment which facilitates requests to add, remove, disable and enable collision objects, while publishing its current state to keep other ROS nodes updated with the latest environment.

### `tesseract_planning_server`
This package contains a planning server supporting asynchronous execution of multiple planning requests.

## Build
- Create a `colcon` workspace and clone this repository into its `src` directory

- Add the dependencies
    ```bash
    cd <colcon_ws>
    vcs import < src/tesseract_ros2/dependencies.repos
    rosdep install --from-paths src -iry
    ```

- Build
    ```bash
    colcon build --symlink-install --cmake-args -DTESSERACT_BUILD_FCL=OFF -DBUILD_RENDERING=OFF -DBUILD_STUDIO=OFF
    ```
