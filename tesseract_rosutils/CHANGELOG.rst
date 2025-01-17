^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_rosutils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.26.0 (2024-11-19)
-------------------
* Update to latest tesseract
* Contributors: Levi Armstrong, Roelof Oomen

0.28.0 (2025-01-17)
-------------------
* Update due to changes with profiles and overrides
* Contributors: Levi Armstrong

0.27.0 (2024-12-01)
-------------------
* Fix plotTrajectory with InstructionPoly
* - Updates to support visualizing online planning example
  - 0.26.0
  - Update changelogs
  - Update rosinstall
* Generate unique node name for ROSPlotting
* - Fix ServicesQoS for older ROS2 versions
  - Add RCLCPP_VERSION_GTE macro for older ROS versions
  - Use RCLCPP_VERSION_GTE instead of header check to determine Humble
* Revert SensorDataQoS and publisher queue sizes
* Clean up QoS settings
* Update to latest tesseract
* Contributors: Roelof Oomen

0.25.0 (2024-09-30)
-------------------
* Add plotToolpath for ToolPath
* Update to support Geometry Type CompoundMesh
* Contributors: Roelof Oomen

0.23.0 (2024-08-21)
-------------------
* Fixes for building on Ubuntu Noble
* Update because TaskComposerProblem was removed
* Node cleanup (`#109 <https://github.com/marip8/tesseract_ros2/issues/109>`_)
  * Add forgotten remove_node calls
  * Use internal_node\_ for all node calls (except parameters) and remove node\_ member
  * Rename node\_ to internal_node\_ to be consistent with e.g. ROSEnvironmentMonitor
  * Change callback group to MutuallyExclusive
  * Fixup
  * Change ROSPlotting to use a SingleThreadedExecutor
  * Change DEFAULT_JOINT_STATES_TOPIC to start with /
* Contributors: Roelof Oomen

0.22.0 (2024-07-11)
-------------------
* - Upgrade tesseract, ros_industrial_cmake_boilerplate and descartes version
  - 0.22.0
* Merge pull request `#104 <https://github.com/tesseract-robotics/tesseract_ros2/issues/104>`_ from marrts/update_stable_0-22
* Fix clang tidy errors in rosutils
* Update to change in task composer node info
* Match tesseract_ros`#233 <https://github.com/tesseract-robotics/tesseract_ros2/issues/233>`_ for tesseract_rosutils/test
* Remove unused deps
* Fix build issues in tesseract_rosutils
* Merge pull request `#95 <https://github.com/tesseract-robotics/tesseract_ros2/issues/95>`_ from marrts/reset_marker_counter
  Add ability to reset marker counter without clearing
* Add ability to reset marker counter without clearing
* - Add description parameter to plotTrajectory functions
  - Add a plotTrajectories function to plot multiple trajectories in one set
  - Minor API change: removed parameter ns from one function, as it was unused but might give the false impression it would be used instead of traj.ns
  - Fixed the clear() function, as it removed all markers except the tool markers
* Merge pull request `#93 <https://github.com/tesseract-robotics/tesseract_ros2/issues/93>`_ from marrts/feat/configurable_trajectory_names
  Pass description field to trajectory objects
* Pass description field to trajectory objects
* Add polygon mesh support
* Simplify thread construction
* Contributors: Roelof Oomen, Tyler Marr

0.21.2 (2024-01-03)
-------------------

0.21.1 (2024-01-03)
-------------------

0.21.0 (2023-11-10)
-------------------

0.20.1 (2023-10-30)
-------------------

0.20.0 (2023-09-30)
-------------------

0.19.0 (2023-09-06)
-------------------
* Update to use tesseract package components
* Contributors: Levi Armstrong

0.18.2 (2023-09-22)
-------------------

0.18.1 (2023-09-20)
-------------------
* Merge pull request `#64 <https://github.com/tesseract-robotics/tesseract_ros2/issues/64>`_ from rjoomen/includecleanup
  Cleanup includes and dependencies
* - Add ament_index_cpp dependency to tesseract_rosutils
  - Remove ament_target_dependencies() from tesseract_ros_examples
* - Link tf2_eigen privately.
  - Remove pluginlib dependency from tesseract_monitoring.
* Foxy build fixes (and a warning fix)
* Cleanup includes and dependencies
* Contributors: Roelof Oomen, Tyler Marr

0.18.0 (2023-07-03)
-------------------
* Changes to support task composer restructure
* Contributors: Levi Armstrong

0.17.0 (2023-06-07)
-------------------

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-10)
-------------------
* Add support for AddTrajectoryLinkCommand
* Contributors: Levi Armstrong

0.15.2 (2023-03-14)
-------------------

0.15.1 (2023-03-05)
-------------------

0.15.0 (2023-03-04)
-------------------
* Fix toMsg and fromMsg for capsule (`#194 <https://github.com/tesseract-robotics/tesseract_ros/issues/194>`_)
* Contributors: Matthew Powelson

0.14.0 (2022-10-23)
-------------------
* Update to use modify allowed collisions command
* Remove planning archive plugin
* Replace tesseract_process_managers with tesseract_task_composer package
* Fix not returning value for message conversion functions
* Add ros conversions for joint map
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------
* Update to use new Poly types in tesseract_planning
* Contributors: Levi Armstrong

0.5.1 (2022-06-21)
------------------

0.5.0 (2022-05-17)
------------------

0.4.4 (2022-05-13)
------------------
* Add new RViz plugins using Tesseract widgets (`#152 <https://github.com/tesseract-robotics/tesseract_ros/issues/152>`_)
* Contributors: Levi Armstrong

0.4.3 (2022-05-03)
------------------
* Update changes with serialization (`#151 <https://github.com/tesseract-robotics/tesseract_ros/issues/151>`_)
  * Update changes with serialization
  * Update rosinstall files
* Contributors: Levi Armstrong

0.4.2 (2022-04-25)
------------------

0.4.1 (2022-04-13)
------------------

0.4.0 (2022-04-08)
------------------
* Update to use monitor interface and clean up environment monitor
* Contributors: Levi Armstrong

0.3.3 (2022-02-22)
------------------

0.3.2 (2022-01-21)
------------------

0.3.1 (2021-12-16)
------------------
* Fix bug in how geometry octree are converted from message and visualized
* Add missing visualization_msgs to tesseract_rosutils CMakelists.txt
* Contributors: Levi Armstrong

0.3.0 (2021-12-06)
------------------
* Update renaming of ContactManagerConfig variables
* Support moving AllowedCollisionMatrix into tesseract_common namespace
* Contributors: Levi Armstrong, Matthew Powelson

0.2.2 (2021-11-30)
------------------

0.2.1 (2021-11-30)
------------------
* Add contact margin data override MODIFY (`#133 <https://github.com/tesseract-robotics/tesseract_ros/issues/133>`_)
  * Add contact margin data override MODIFY
  * Update rosinstall tesseract hash
* Cleanup CMakeLists.txt
* Contributors: Levi Armstrong

0.2.0 (2021-11-04)
------------------
* Update due to changes with contact manager plugins
* Update to Joint and Kinematic group (`#125 <https://github.com/tesseract-robotics/tesseract_ros/issues/125>`_)
* Remove References to Deprecated Tesseract_geometry Functions (`#124 <https://github.com/tesseract-robotics/tesseract_ros/issues/124>`_)
* Update online planner to latest changes in trajopt ifopt package (`#119 <https://github.com/tesseract-robotics/tesseract_ros/issues/119>`_)
  Co-authored-by: ben-greenberg <benrgreenberg@gmail.com>
  Co-authored-by: ben <ben.greenberg@swri.org>
* Update Tesseract removed deprecated code
* Clean up environment monitor and interface
* Update new tesseract_srdf package
* Update due to switching to boost serialization
* Fix trail visualization and fix processing of empty commands message
* Update for changes with CollisionMarginData
* Clang format
* Add TaskInfo message
* Include joint state in to/from msg utils for Environment
* Add optional Environment to EnvironmentState.msg
* Change TesseractState.msg to EnvironmentState.msg
* Switch plotting of toolpath to use marker array to support namespaces
* Add replace link and joint support (`#85 <https://github.com/tesseract-robotics/tesseract_ros/issues/85>`_)
* Update to latest tesseract_environment changes and fix online planning example
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update to leverage new visualizaton interface
* Move all packages out of tesseract_ros sub directory
* Contributors: DavidMerzJr, Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
