# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS 2 integration layer for the Tesseract motion planning framework. Bridges Tesseract core libraries to ROS 2 nodes, services, actions, and messages.

## Architecture

### Packages

- **tesseract_msgs** — ROS message, service, and action definitions
- **tesseract_rosutils** — Conversion utilities between ROS types and Tesseract native types
- **tesseract_monitoring** — `ROSEnvironmentMonitor` (URDF/SRDF loading, services, TF2) and `ContactMonitor` (collision monitoring, RViz markers)
- **tesseract_planning_server** — `GetMotionPlan` action server wrapping `TaskComposerServer`
- **tesseract_ros_examples** / **tesseract_qt_ros** / **tesseract_rviz** — Examples, Qt-ROS bridge, RViz plugins

### Non-obvious integration details

- `GetEnvironmentInformation` uses bitwise OR flags for selective data retrieval (links=1, joints=2, transforms=4, etc.)
- Planning server accepts XML-serialized task descriptions via the `GetMotionPlan` action
