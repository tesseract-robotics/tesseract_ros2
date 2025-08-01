^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_rviz
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.31.0 (2025-07-08)
-------------------
* Fix tesseract_rviz to export targets and depends
* Use new tesseract_qt EventType
* Fix scaling issue of capsule visualization
* Contributors: Levi Armstrong, Roelof Oomen

0.30.0 (2025-04-23)
-------------------

0.29.2 (2025-03-26)
-------------------
* Update component info usage
* Contributors: Levi Armstrong

0.29.1 (2025-03-21)
-------------------

0.29.0 (2025-03-21)
-------------------
* - Improve documentation of Trajectory.msg
  - Simplify initial state conversion in plotTrajectory()
  - Remove unused description from Trajectory.msg (replaced by the description of the CompositeInstruction or by joint_trajectories_description)
  - Fix use of description in plotTrajectory()
  - Add uuid to plotTrajectories() function
  - Add plotTrajectories() with Commands
  - Fix tesseractJointTrajectoryCallback for joint_trajectories
* Remove TransformBroadcaster from CurrentStateMonitor and reintroduce in ROSEnvironmentMonitor (`#144 <https://github.com/tesseract-robotics/tesseract_ros2/issues/144>`_)
* Update to leverage std::filesystem
* Contributors: Levi Armstrong, Roelof Oomen

0.28.2 (2025-01-29)
-------------------
* Fix failed to parse urdf error message on launch
* Contributors: Levi Armstrong

0.28.1 (2025-01-24)
-------------------
* Add missing floating joint components (`#137 <https://github.com/tesseract-robotics/tesseract_ros2/issues/137>`_)
* Contributors: Roelof Oomen

0.28.0 (2025-01-17)
-------------------

0.27.0 (2024-12-01)
-------------------
* - Updates to support visualizing online planning example
  - 0.26.0
  - Update changelogs
  - Update rosinstall
* Fix QoS profile for subscription
* - Fix ServicesQoS for older ROS2 versions
  - Add RCLCPP_VERSION_GTE macro for older ROS versions
  - Use RCLCPP_VERSION_GTE instead of header check to determine Humble
* Revert SensorDataQoS and publisher queue sizes
* Clean up QoS settings
* Contributors: Roelof Oomen

0.26.0 (2024-11-19)
-------------------

0.25.0 (2024-09-30)
-------------------
* Update to support Geometry Type CompoundMesh
* Contributors: Levi Armstrong, Roelof Oomen

0.23.0 (2024-08-21)
-------------------
* Fixes for building on Ubuntu Noble
* Contributors: Roelof Oomen

0.22.0 (2024-07-11)
-------------------
* - Upgrade tesseract, ros_industrial_cmake_boilerplate and descartes version
  - 0.22.0
* Fix adding Octree to entity_container
* Match tesseract_ros`#233 <https://github.com/tesseract-robotics/tesseract_ros2/issues/233>`_ for tesseract_rviz
* Remove more commented includes
* Fix build issues in tesseract_rviz
* Merge pull request `#93 <https://github.com/tesseract-robotics/tesseract_ros2/issues/93>`_ from marrts/feat/configurable_trajectory_names
  Pass description field to trajectory objects
* Pass description field to trajectory objects
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
* Fix Qt SendEvent Calls
* Contributors: Levi Armstrong

0.20.0 (2023-09-30)
-------------------
* Fix saving EnvMonitorSnapshotTopic
* Contributors: Roelof

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
* Cleanup includes and dependencies
* Merge pull request `#63 <https://github.com/tesseract-robotics/tesseract_ros2/issues/63>`_ from rjoomen/clockfix
  Fix clock comparison errors
* - Initializing time variables to prevent clock comparision errors. Closes `#50 <https://github.com/tesseract-robotics/tesseract_ros2/issues/50>`_.
  - Cleaned up includes.
* Contributors: Roelof Oomen, Tyler Marr

0.18.0 (2023-07-03)
-------------------

0.17.0 (2023-06-07)
-------------------
* Update based on changes to tesseract_qt to support studio plugins (`#212 <https://github.com/tesseract-robotics/tesseract_ros/issues/212>`_)
* Contributors: Levi Armstrong

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------
* Add environment snapshot viewing to environment monitor
* Contributors: Levi Armstrong

0.16.0 (2023-04-10)
-------------------

0.15.2 (2023-03-14)
-------------------
* Fix tesseract rviz export libraries
* Contributors: Levi Armstrong

0.15.1 (2023-03-05)
-------------------

0.15.0 (2023-03-04)
-------------------
* Update to leverage tesseract_qt event filters (`#199 <https://github.com/tesseract-robotics/tesseract_ros/issues/199>`_)
* Contributors: Levi Armstrong

0.14.0 (2022-10-23)
-------------------
* Update to use modify allowed collisions command
* Remove planning archive plugin
* Remove legacy rviz plugins
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------
* Fix ogre getAABB
* Disable publishing tf in environment monitor properties
* Update to use new Poly types in tesseract_planning
* Update to latest tesseract
* Contributors: Levi Armstrong

0.5.1 (2022-06-21)
------------------
* Add cartesian interactive marker support
* Add joint interactive marker to manipulation widget
* Contributors: Levi Armstrong

0.5.0 (2022-05-17)
------------------
* Update to use tesseract_qt (`#154 <https://github.com/tesseract-robotics/tesseract_ros/issues/154>`_)
* Contributors: Levi Armstrong

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
* Contributors: Levi Armstrong

0.3.0 (2021-12-06)
------------------
* Support moving AllowedCollisionMatrix into tesseract_common namespace
* Contributors: Matthew Powelson

0.2.2 (2021-11-30)
------------------

0.2.1 (2021-11-30)
------------------
* Cleanup CMakeLists.txt
* Contributors: Levi Armstrong

0.2.0 (2021-11-04)
------------------
* Update due to changes with contact manager plugins
* Improve manipulation widget support for external positioners
* Fix manipulator widget updating config segfault
* Update to Joint and Kinematic group (`#125 <https://github.com/tesseract-robotics/tesseract_ros/issues/125>`_)
* Remove References to Deprecated Tesseract_geometry Functions (`#124 <https://github.com/tesseract-robotics/tesseract_ros/issues/124>`_)
* Update Tesseract removed deprecated code
* Clean up environment monitor and interface
* Add online example rviz config and fix trajectory display after disable
* Update due to switching to boost serialization
* Remove use of isWithinLimits and use satisfiesPositionLimits
* Fix trail visualization and fix processing of empty commands message
* Clang format
* Check for empty xml in PlanningRequestArchiveViewer
* Add optional Environment to EnvironmentState.msg
* Change TesseractState.msg to EnvironmentState.msg
* Refactor RVIZ trajectory widget to allow it to be reused
* Updates to PlanningResponseArchive viewer
* Update to new forward and inverse kinematics interface
* Updates to support fromXML templates
* Add replace link and joint support (`#85 <https://github.com/tesseract-robotics/tesseract_ros/issues/85>`_)
* Update to latest tesseract_environment changes and fix online planning example
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update to leverage new visualizaton interface
* Move all packages out of tesseract_ros sub directory
* Contributors: DavidMerzJr, Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
* WIP: Move ROS package into sub folder
* Isolate tesseract_collision namespace
* Switch to using built in Collision Shapes
* Fix formatting using clang
* Fix warnings in unit tests
* Add additional compiler warning options
* Updated bullet_ros to not build unit tests; added line for installation of plugin XML files in Rviz package
* Eigen Alignment fixes
* Add monitoring of joint state topic to tesseract state display
* Merge pull request `#20 <https://github.com/tesseract-robotics/tesseract_ros/issues/20>`_ from Levi-Armstrong/feature/Isometry3d
  switch from using affine3d to isometry3d
* switch from using affine3d to isometry3d
* Move tesseract into its own repository
* Contributors: Levi, Levi Armstrong, Matthew Powelson, mripperger
