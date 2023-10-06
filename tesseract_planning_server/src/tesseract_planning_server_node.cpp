/**
 * @file tesseract_planning_server_node.cpp
 * @brief The Tesseract planning server node
 *
 * @author Levi Armstrong
 * @date August 18, 2020
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_planning_server/tesseract_planning_server.h>

using namespace tesseract_environment;
using tesseract_planning_server::TesseractPlanningServer;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_SEMANTIC_PARAM = "robot_description_semantic";

static std::shared_ptr<tesseract_planning_server::TesseractPlanningServer> planning_server;

void updateCacheCallback() { planning_server->getEnvironmentCache().refreshCache(); }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tesseract_planning_server");

  // Defaults
  bool publish_environment{ false };
  long cache_size{ 5 };
  double cache_refresh_rate{ 0.1 };

  std::string monitor_namespace = node->declare_parameter("monitor_namespace", "");
  if (monitor_namespace.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter monitor_namespace!");
    return 1;
  }
  std::string monitored_namespace = node->declare_parameter("monitored_namespace", "");
  std::string robot_description = node->declare_parameter(ROBOT_DESCRIPTION_PARAM, "");
  if (robot_description.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter robot_description!");
    return 1;
  }
  std::string robot_description_semantic = node->declare_parameter(ROBOT_DESCRIPTION_SEMANTIC_PARAM, "");
  if (robot_description_semantic.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Missing required parameter robot_description_semantic!");
    return 1;
  }
  publish_environment = node->declare_parameter("publish_environment", publish_environment);
  cache_size = node->declare_parameter("cache_size", cache_size);
  cache_refresh_rate = node->declare_parameter("cache_refresh_rate", cache_refresh_rate);
  std::string task_composer_config = node->declare_parameter("task_composer_config", "");

  planning_server = std::make_shared<TesseractPlanningServer>(node, ROBOT_DESCRIPTION_PARAM, monitor_namespace);

  planning_server->getEnvironmentCache().setCacheSize(cache_size);

  if (publish_environment)
    planning_server->getEnvironmentMonitor().startPublishingEnvironment();

  if (!monitored_namespace.empty())
    planning_server->getEnvironmentMonitor().startMonitoringEnvironment(monitored_namespace);

  if (!task_composer_config.empty())
  {
    tesseract_common::fs::path config(task_composer_config);
    planning_server->getTaskComposerServer().loadConfig(config);
  }

  // TODO: Rate seems in seconds, whereas this should be in Hz (otherwise it should be called interval)
  auto update_cache =
      node->create_wall_timer(std::chrono::duration<double>(cache_refresh_rate), [] { updateCacheCallback(); });

  RCLCPP_INFO(node->get_logger(), "Planning Server Running!");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
