/**
 * @file online_planning_example_node.cpp
 * @brief This example demonstrates using trajopt to plan in an "online" manner. As the environment is changed (using
 * the joint state publisher), the system will attempt to avoid collisions with the moving object and follow the moving
 * target
 *
 * Note: If the target moves too quickly the solver can get stuck in an infeasible point. That is the nature of how the
 * solver is working. Higher level intelligence, a larger step size, or changing the target to a cost can solve that
 * problem.
 *
 * @author Matthew Powelson
 * @date June 9, 2020
 * @version TODO
 * @bug No known bugs
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
#include <tesseract_examples/online_planning_example.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <std_srvs/srv/set_bool.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_scene_graph/graph.h>

using namespace tesseract_examples;
using namespace tesseract_rosutils;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief RViz Example Namespace */
const std::string EXAMPLE_MONITOR_NAMESPACE = "tesseract_ros_examples";

/** @brief Dynamic object joint states topic */
const std::string DYNAMIC_OBJECT_JOINT_STATE = tesseract_monitoring::DEFAULT_JOINT_STATES_TOPIC;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("online_planning_example_node");

  // Get ROS Parameters
  bool plotting = node->declare_parameter("plotting", true);
  bool rviz = node->declare_parameter("rviz", true);

  long steps = node->declare_parameter("steps", 12);
  double box_size = node->declare_parameter("box_size", 0.01);
  bool update_start_state = node->declare_parameter("update_start_state", false);
  bool use_continuous = node->declare_parameter("use_continuous", false);

  // Initial setup
  std::string urdf_xml_string = node->declare_parameter(ROBOT_DESCRIPTION_PARAM, "");
  std::string srdf_xml_string = node->declare_parameter(ROBOT_SEMANTIC_PARAM, "");

  auto env = std::make_shared<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
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

  OnlinePlanningExample example(env, plotter, static_cast<int>(steps), box_size, update_start_state, use_continuous);

  auto fn1 = [&example](std_srvs::srv::SetBool::Request::SharedPtr req,   // NOLINT
                        std_srvs::srv::SetBool::Response::SharedPtr res)  // NOLINT
  {
    example.toggleRealtime(req->data);
    res->success = true;
    return true;
  };

  auto fn2 = [&example](const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)  // NOLINT
  { example.updateState(joint_state->name, joint_state->position); };

  // Set up ROS interfaces
  auto joint_state_subscriber_ =
      node->create_subscription<sensor_msgs::msg::JointState>(DYNAMIC_OBJECT_JOINT_STATE, rclcpp::QoS(20), fn2);

  auto toggle_realtime_service = node->create_service<std_srvs::srv::SetBool>("toggle_realtime", fn1);

  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(5.0)));

  if (!example.run())
  {
    RCLCPP_ERROR(node->get_logger(), "OnlinePlanningExample failed");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "OnlinePlanningExample successful");
  }

  spinner.join();
}
