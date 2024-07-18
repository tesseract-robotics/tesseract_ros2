^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_monitoring
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.22.0 (2024-07-11)
-------------------
* - Upgrade tesseract, ros_industrial_cmake_boilerplate and descartes version
  - 0.22.0
* Merge pull request `#104 <https://github.com/tesseract-robotics/tesseract_ros2/issues/104>`_ from marrts/update_stable_0-22
* Switch to bitwise comparison
* Move publish_environment_frequency\_ initialization to header
* Fix code quality issues in tesseract_monitoring
* Additional cleanups
* Add fwd to environment_monitor_interface
* Remove commented include
* Fixed a missed fwd declaration bug in tesseract_monitoring
* Fix build issues in tesseract_monitoring
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

0.18.2 (2023-09-22)
-------------------

0.18.1 (2023-09-20)
-------------------
* Merge pull request `#64 <https://github.com/tesseract-robotics/tesseract_ros2/issues/64>`_ from rjoomen/includecleanup
  Cleanup includes and dependencies
* - Link tf2_eigen privately.
  - Remove pluginlib dependency from tesseract_monitoring.
* Foxy build fixes (and a warning fix)
* Cleanup includes and dependencies
* Merge pull request `#63 <https://github.com/tesseract-robotics/tesseract_ros2/issues/63>`_ from rjoomen/clockfix
  Fix clock comparison errors
* - Initializing time variables to prevent clock comparision errors. Closes `#50 <https://github.com/tesseract-robotics/tesseract_ros2/issues/50>`_.
  - Cleaned up includes.
* Fix environment monitor node to not publish tf if monitoring another environment
* Contributors: Roelof Oomen, Tyler Marr

0.18.0 (2023-07-03)
-------------------

0.17.0 (2023-06-07)
-------------------
* Fix shadowing outer variable (`#215 <https://github.com/tesseract-robotics/tesseract_ros/issues/215>`_)
* Contributors: Roelof

0.16.2 (2023-04-28)
-------------------

0.16.1 (2023-04-11)
-------------------

0.16.0 (2023-04-10)
-------------------
* Update to leverge ContactResultMap class (`#205 <https://github.com/tesseract-robotics/tesseract_ros/issues/205>`_)
* Contributors: Levi Armstrong

0.15.2 (2023-03-14)
-------------------

0.15.1 (2023-03-05)
-------------------

0.15.0 (2023-03-04)
-------------------

0.14.0 (2022-10-23)
-------------------
* Remove use of deprecated items
* Contributors: Levi Armstrong

0.6.0 (2022-08-25)
------------------

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

0.4.2 (2022-04-25)
------------------

0.4.1 (2022-04-13)
------------------
* Fix handling of wait(0) in environment monitor interface
* Contributors: Levi Armstrong

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
* Add ability to disable links when launching contact monitor
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
* Update to Joint and Kinematic group (`#125 <https://github.com/tesseract-robotics/tesseract_ros/issues/125>`_)
* Rename tesseract_monitor_interface to environment_monitor_interface
* Clean up environment monitor and interface
* Update new tesseract_srdf package
* Clang format
* Change TesseractState.msg to EnvironmentState.msg
* Update to latest tesseract_environment changes and fix online planning example
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Update to leverage new visualizaton interface
* Move all packages out of tesseract_ros sub directory
* Contributors: Levi Armstrong, Levi-Armstrong, Matthew Powelson

0.1.0 (2020-12-02)
------------------
* WIP: Move ROS package into sub folder
* Semi-Isolate Tesseract Kinematics
* Isolate tesseract_collision namespace
* Switch to using built in Collision Shapes
* Clang formatting changes
* Added service server to tesseract environment monitor for updating the environment
* Merge branch 'kinetic-devel' into acm_fixes
* Added installation of tesseract_monitoring launch files to CMakeLists
* Fix formatting using clang
* Fix warnings in unit tests
* Add additional compiler warning options
* Implement topic subscriber for updating collision monitor environment
* Implement synchronous "compute_contact_reports" service in contact_monitor.cpp
* Fixed typo 'constacts' in ContactResultVector.msg
* Merge pull request `#41 <https://github.com/tesseract-robotics/tesseract_ros/issues/41>`_ from Levi-Armstrong/issue/FixMultiLayerCompoundShape
  Fix use of multi layer compound shape
  Fix/add cmake install commands
* Fix cmake install commands
* Merge pull request `#40 <https://github.com/tesseract-robotics/tesseract_ros/issues/40>`_ from Levi-Armstrong/feature/RemoveContactRequestStruct
  Refractor out ContactRequest type
* Refractor out ContactRequest type
* Merge pull request `#26 <https://github.com/tesseract-robotics/tesseract_ros/issues/26>`_ from Levi-Armstrong/issue/FixContactMonitor
  Update contact monitor to use the latest version
* Fix the contact monitor to use the new contact managers
* Merge pull request `#20 <https://github.com/tesseract-robotics/tesseract_ros/issues/20>`_ from Levi-Armstrong/feature/Isometry3d
  switch from using affine3d to isometry3d
* switch from using affine3d to isometry3d
* Merge pull request `#15 <https://github.com/tesseract-robotics/tesseract_ros/issues/15>`_ from Levi-Armstrong/feature/largeDataSetTest
  Restructure Collision Checking for Performance Improvements
* Run clang-format
* Restructure Collision Checking for Performance Improvements
* Move tesseract into its own repository
* Contributors: Alessio Rocchi, John Wason, Levi, Levi Armstrong, mripperger
