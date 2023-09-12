/**
 * @file contact_monitor_node.cpp
 * @brief Node that instantiates a contact_monitor
 *
 * @author David Merz, Jr.
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_monitoring/contact_monitor.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_rosutils/utils.h>

// Stuff for the contact monitor
/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

const static double DEFAULT_CONTACT_DISTANCE = 0.1;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tesseract_contact_monitoring");

  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  tesseract_srdf::SRDFModel::Ptr srdf_model;

  std::string monitor_namespace = node->declare_parameter("monitor_namespace", "");
  if (monitor_namespace.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter monitor_namespace!");
    return 1;
  }

  std::string monitored_namespace = node->declare_parameter("monitored_namespace", "");
  std::string robot_description = node->declare_parameter(ROBOT_DESCRIPTION_PARAM, "");
  std::string robot_description_semantic = node->declare_parameter(ROBOT_SEMANTIC_PARAM, "");
  std::string joint_state_topic =
      node->declare_parameter("joint_state_topic", tesseract_monitoring::DEFAULT_JOINT_STATES_TOPIC);
  bool publish_environment = node->declare_parameter("publish_environment", false);
  bool publish_markers = node->declare_parameter("publish_markers", false);

  // Initial setup
  if (robot_description.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to find parameter: %s", ROBOT_DESCRIPTION_PARAM.c_str());
    return -1;
  }

  if (robot_description_semantic.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to find parameter: %s", ROBOT_SEMANTIC_PARAM.c_str());
    return -1;
  }

  auto env = std::make_unique<tesseract_environment::Environment>();
  auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env->init(robot_description, robot_description_semantic, locator))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize environment.");
    return -1;
  }

  double contact_distance = node->declare_parameter("contact_distance", DEFAULT_CONTACT_DISTANCE);

  std::vector<std::string> monitored_link_names;
  std::string monitored_link_names_str = node->declare_parameter("monitor_links", "");
  if (!monitored_link_names_str.empty())
  {
    boost::split(monitored_link_names, monitored_link_names_str, boost::is_any_of(" "));
  }

  if (monitored_link_names.empty())
    monitored_link_names = env->getLinkNames();

  std::vector<std::string> disabled_link_names;
  std::string disabled_link_names_str = node->declare_parameter("disabled_links", "");
  if (!disabled_link_names_str.empty())
  {
    boost::split(disabled_link_names, disabled_link_names_str, boost::is_any_of(" "));
  }

  for (const auto& disabled_link : disabled_link_names)
  {
    if (disabled_link_names_str.empty())
      disabled_link_names_str = disabled_link;
    else
      disabled_link_names_str += (", " + disabled_link);
  }

  RCLCPP_INFO(node->get_logger(), "DISABLED_LINKS: %s", disabled_link_names_str.c_str());

  auto pred = [&disabled_link_names](const std::string& key) -> bool {
    return std::find(disabled_link_names.begin(), disabled_link_names.end(), key) != disabled_link_names.end();
  };

  monitored_link_names.erase(std::remove_if(monitored_link_names.begin(), monitored_link_names.end(), pred),
                             monitored_link_names.end());

  for (const auto& monitor_link : monitored_link_names)
  {
    if (monitored_link_names_str.empty())
      monitored_link_names_str = monitor_link;
    else
      monitored_link_names_str += (", " + monitor_link);
  }

  RCLCPP_INFO(node->get_logger(), "MONITORED_LINKS: %s", monitored_link_names_str.c_str());

  int contact_test_type = node->declare_parameter("contact_test_type", 2);

  if (contact_test_type < 0 || contact_test_type > 3)
  {
    RCLCPP_WARN(node->get_logger(), "Request type must be 0, 1, 2 or 3. Setting to 2(ALL)!");
    contact_test_type = 2;
  }
  tesseract_collision::ContactTestType type = static_cast<tesseract_collision::ContactTestType>(contact_test_type);

  tesseract_monitoring::ContactMonitor cm(monitor_namespace,
                                          std::move(env),
                                          node,
                                          monitored_link_names,
                                          disabled_link_names,
                                          type,
                                          contact_distance,
                                          joint_state_topic);

  if (publish_environment)
    cm.startPublishingEnvironment();

  if (!monitored_namespace.empty())
    cm.startMonitoringEnvironment(monitored_namespace);

  if (publish_markers)
    cm.startPublishingMarkers();

  std::thread t(&tesseract_monitoring::ContactMonitor::computeCollisionReportThread, &cm);

  RCLCPP_INFO(node->get_logger(), "Contact Monitor Running!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
