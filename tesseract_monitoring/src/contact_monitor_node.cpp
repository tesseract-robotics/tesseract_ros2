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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/srdf/srdf_model.h>
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

  tesseract::scene_graph::SceneGraph::Ptr scene_graph;
  tesseract::srdf::SRDFModel::Ptr srdf_model;

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

  auto env = std::make_unique<tesseract::environment::Environment>();
  auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  if (!env->init(robot_description, robot_description_semantic, locator))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize environment.");
    return -1;
  }

  double contact_distance = node->declare_parameter("contact_distance", DEFAULT_CONTACT_DISTANCE);

  // Link parameters are space separated lists of link names
  auto declare_link_ids_parameter = [&node](const std::string& name) {
    std::vector<std::string> link_names;
    const std::string value = node->declare_parameter(name, "");
    if (!value.empty())
      boost::split(link_names, value, boost::is_any_of(" "));

    return tesseract::common::toIds<tesseract::common::LinkId>(link_names);
  };

  std::vector<tesseract::common::LinkId> monitored_link_ids = declare_link_ids_parameter("monitor_links");
  if (monitored_link_ids.empty())
    monitored_link_ids = env->getLinkIds();

  std::vector<tesseract::common::LinkId> disabled_link_ids = declare_link_ids_parameter("disabled_links");

  RCLCPP_INFO(node->get_logger(),
              "DISABLED_LINKS: %s",
              boost::algorithm::join(tesseract::common::toNames(disabled_link_ids), ", ").c_str());

  auto pred = [&disabled_link_ids](const tesseract::common::LinkId& key) -> bool {
    return std::find(disabled_link_ids.begin(), disabled_link_ids.end(), key) != disabled_link_ids.end();
  };

  monitored_link_ids.erase(std::remove_if(monitored_link_ids.begin(), monitored_link_ids.end(), pred),
                           monitored_link_ids.end());

  RCLCPP_INFO(node->get_logger(),
              "MONITORED_LINKS: %s",
              boost::algorithm::join(tesseract::common::toNames(monitored_link_ids), ", ").c_str());

  auto contact_test_type = node->declare_parameter("contact_test_type", 2);

  if (contact_test_type < 0 || contact_test_type > 3)
  {
    RCLCPP_WARN(node->get_logger(), "Request type must be 0, 1, 2 or 3. Setting to 2(ALL)!");
    contact_test_type = 2;
  }
  auto type = static_cast<tesseract::collision::ContactTestType>(contact_test_type);

  tesseract_monitoring::ContactMonitor cm(monitor_namespace,
                                          std::move(env),
                                          node,
                                          std::move(monitored_link_ids),
                                          std::move(disabled_link_ids),
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
