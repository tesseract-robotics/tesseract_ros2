/**
 * @file puzzle_piece_auxillary_axes_example_node.cpp
 * @brief Puzzle piece auxillary axes example node
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <tesseract_examples/puzzle_piece_auxillary_axes_example.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

#include <tesseract/common/resource_locator.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/graph.h>

using namespace tesseract::examples;
using namespace tesseract_rosutils;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("puzzle_piece_auxillary_axes_example_node");

  // Get ROS Parameters
  bool plotting = node->declare_parameter("plotting", true);
  bool rviz = node->declare_parameter("rviz", true);
  bool ifopt = node->declare_parameter("ifopt", false);
  bool debug = node->declare_parameter("debug", false);

  // Initial setup
  std::string urdf_xml_string = node->declare_parameter(ROBOT_DESCRIPTION_PARAM, "");
  std::string srdf_xml_string = node->declare_parameter(ROBOT_SEMANTIC_PARAM, "");

  if (ifopt)
  {
    RCLCPP_INFO(node->get_logger(), "Using TrajOpt Ifopt!");
  }

  auto env = std::make_shared<tesseract::environment::Environment>();
  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  if (!env->init(urdf_xml_string, srdf_xml_string, locator))
    exit(1);

  std::thread spinner{ [node]() { rclcpp::spin(node); } };

  // Create monitor
  auto monitor = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node, env, EXAMPLE_MONITOR_NAMESPACE);
  if (rviz)
    monitor->startPublishingEnvironment();

  ROSPlottingPtr plotter;
  if (plotting)
    plotter = std::make_shared<ROSPlotting>(env->getSceneGraph()->getRoot());

  PuzzlePieceAuxillaryAxesExample example(env, plotter, ifopt, debug);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(5.0)));

  if (!example.run())
  {
    RCLCPP_ERROR(node->get_logger(), "PuzzlePieceAuxillaryAxesExample failed");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "PuzzlePieceAuxillaryAxesExample successful");
  }

  spinner.join();
}
